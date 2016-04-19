'''
Created on Jan 19, 2016

@author: ymeng
'''
import matplotlib.pyplot as plt
from preprocess import preprocess
import classifier
import myo_state
from build_mdp import BuildMDP
import cPickle as pickle
import argparse
import os
import numpy as np
from align_signal import align_signal

def main():
    parser = argparse.ArgumentParser(description="Do something.")
    parser.add_argument('-s', '--samples', default=1, type=int)
    parser.add_argument('-n', '--clusters', default=None, type=int)
    
    args = parser.parse_args()
    
    for i in range(args.samples):
        data_i, EMG_max_i, EMG_min_i, GYRO_max_i  = preprocess(os.path.join('../../data/work/', str(i)), update_extremes=True)
        emg_i = data_i[:, 1:9]
        imu_i = data_i[:, 9:]
        if i == 0:
            emg_demos = emg_i
            imu_demos = imu_i
            EMG_MAX = EMG_max_i
            EMG_MIN = EMG_min_i
            GYRO_MAX = GYRO_max_i
        else:
            imu_i_a = align_signal(imu_demos, imu_i, w=5, has_time=False)
            imu_demos += imu_i_a
            EMG_MAX += EMG_max_i
            EMG_MIN += EMG_min_i
            GYRO_MAX += GYRO_max_i
    
    # get average                        
    imu_demos = imu_demos/args.samples
    EMG_MAX = EMG_MAX/args.samples
    EMG_MIN = EMG_MIN/args.samples
    GYRO_MAX = GYRO_MAX/args.samples
    print "EMG_MAX", EMG_MAX
    print "EMG_MIN", EMG_MIN
    print "GYRO_MAX", GYRO_MAX
    print "# data points", len(imu_demos)
        
#        imu = data[:, -4:]
    np.savetxt('../data/imu_sample.dat', imu_demos, delimiter=',')
    
    emg_cluster = classifier.SignalCluster(emg_demos, n_clusters=8)
    
    N = imu_demos.shape[0]
    time = np.arange(1,N+1).reshape(N,1)/10.0 # use seconds as feature
    observations = np.hstack((time, 1*emg_demos, imu_demos))
    #print observations.shape
    
    if args.clusters is None:
        N = observations.shape[0]
        low_limit = N/20 # 0.5 states per second
        high_limit = 2*N/20 # 1.5 states per second
        scores = {}
        for n in range(low_limit, high_limit):
            state_cluster = classifier.SignalCluster(observations, n)
            score = state_cluster.evaluate(observations, state_cluster.labels)
            scores[score] = n
            print '# clusters, score', n, score
        
        max_score = max(scores.keys())
        n_clusters = scores[max_score]
        state_cluster = classifier.SignalCluster(observations, n_clusters)
    else:
        state_cluster = classifier.SignalCluster(observations, args.clusters)
        
    #plt.plot(emg)
    plt.figure()
    plt.plot(imu_demos)
    plt.plot(state_cluster.labels, 'r*')
    plt.figure()
    plt.plot(emg_demos)
    #np.savetxt("imu_demos.dat", imu_demos, delimiter=',')
    np.savetxt("emg_demos.dat", emg_demos, delimiter=',')
    
    #plt.plot(emg_cluster.labels, 'go')
    plt.show(block=False)
    
    builder = BuildMDP(actionsData=emg_cluster.labels, statesData=state_cluster.labels)
    print "path", builder.path
    print "policy", builder.Pi
    with open('../data/state_sequence.dat', 'w') as f:
        for item in builder.path:
            f.write('s'+str(item)+'\n')
    
    
    state_classifier = classifier.SignalClassifier(n_neighbors=5)
    state_classifier.train(observations[:,1:], state_cluster.labels, trainingFile=None)
    pickle.dump((state_classifier, EMG_MAX, EMG_MIN, GYRO_MAX), open('../data/state_classifier.pkl', 'wb'))
        
    args = {"give_prompt": True,
            "EMG_MAX": EMG_MAX,
            "EMG_MIN": EMG_MIN,
            "GYRO_MAX": GYRO_MAX
            }
    progress = myo_state.Progress(classifier=state_classifier, **args)
    

if __name__ == '__main__':
    main()
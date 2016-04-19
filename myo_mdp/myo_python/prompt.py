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

TIME_WEIGHT = 0.05
EMG_WEIGHT = 1

def evaluate(emg_labels, state_labels, mdp_builder):
    """Baseline of performance
    """
    total_reward = 0
    N = len(emg_labels)
    print "Number of points: ", N
    for i in range(N-1):
        s = state_labels[i]
        a = emg_labels[i]
        s_next = state_labels[i+1]
        total_reward += mdp_builder.getReward(a, s, s_next)
        
    return total_reward
        
def main():
    parser = argparse.ArgumentParser(description="Do something.")
    parser.add_argument('-s', '--samples', default=1, type=int)
    parser.add_argument('-n', '--clusters', default=None, type=int)
    parser.add_argument('-i', '--id', default='user', type=str)
    parser.add_argument('-u', '--used', default=False, action="store_true") # no need to train the model
    
    
    args = parser.parse_args()
    
    if args.used:
        mdp = pickle.load(open('../data/mdp.pkl'))
        args = {"give_prompt": True,
                "mdp": mdp,
                "id": args.id
                } 
        
        progress = myo_state.Progress(classifier_pkl='../data/state_classifier.pkl', **args)
        return
       
    
    
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
    n = len(imu_demos)
    Time = np.arange(n).reshape((n,1))
    
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
    
    observations = np.hstack((EMG_WEIGHT*emg_demos, imu_demos))
    timed_observations = np.hstack((TIME_WEIGHT*Time, EMG_WEIGHT*emg_demos, imu_demos))
    #print observations.shape
    
    if args.clusters is None:
        N = observations.shape[0]
        low_limit = N/20 # 0.5 states per second
        high_limit = 2*N/20 # 1.5 states per second
        scores = {}
        for n in range(low_limit, high_limit):
            state_cluster = classifier.SignalCluster(timed_observations, n)
            score = state_cluster.evaluate(timed_observations, state_cluster.labels)
            scores[score] = n
            print '# clusters, score', n, score
        
        max_score = max(scores.keys())
        n_clusters = scores[max_score]
        state_cluster = classifier.SignalCluster(timed_observations, n_clusters)
    else:
        state_cluster = classifier.SignalCluster(timed_observations, args.clusters)
        
    plt.figure()
    plt.plot(imu_demos)
    plt.plot(state_cluster.labels, '*')
    #plt.figure()
    #plt.plot(emg_demos)
    np.savetxt("imu_demos.dat", imu_demos, delimiter=',')
    np.savetxt("emg_demos.dat", emg_demos, delimiter=',')
    
    #plt.plot(emg_cluster.labels, 'go')
    plt.show(block=False)
    
    builder = BuildMDP(actionsData=emg_cluster.labels, statesData=state_cluster.labels)
    pickle.dump(builder, open('../data/mdp.pkl', 'wb'))
    
    print "path", builder.path
    print "policy", builder.Pi
    with open('../data/state_sequence.dat', 'w') as f:
        for item in builder.path:
            f.write('s'+str(item)+'\n')
    print "expected actions: ", emg_cluster.labels
    print "expected states: ", state_cluster.labels
    baseline = evaluate(emg_cluster.labels, state_cluster.labels, builder)
    print "baseline performance: ", baseline
    
    
    state_classifier = classifier.SignalClassifier(n_neighbors=5)
    state_classifier.train(observations, state_cluster.labels, trainingFile=None)
    pickle.dump((state_classifier, EMG_MAX, EMG_MIN, GYRO_MAX, baseline), open('../data/state_classifier.pkl', 'wb'))
    
    action_classifier = classifier.SignalClassifier(n_neighbors=5)
    action_classifier.train(emg_demos, emg_cluster.labels, trainingFile=None)
    pickle.dump(action_classifier, open('../data/action_classifier.pkl', 'wb'))
        
    args = {"give_prompt": True,
            "EMG_MAX": EMG_MAX,
            "EMG_MIN": EMG_MIN,
            "GYRO_MAX": GYRO_MAX,
            "mdp": builder,
            "baseline": baseline,
            "id": args.id
            }
    progress = myo_state.Progress(classifier=state_classifier, **args)
    

if __name__ == '__main__':
    main()
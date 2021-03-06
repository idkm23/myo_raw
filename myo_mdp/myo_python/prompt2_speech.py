#!/usr/bin/env python

'''
Created on Mar 24, 2016

@author: ymeng
'''
import matplotlib.pyplot as plt
from preprocess import preprocess
import classifier
import myo_state2
from myo_demo2 import MyoDemo2
from build_mdp import BuildMDP
import cPickle as pickle
import argparse
import os
import sys
import numpy as np
from align_signal import align_signal
import subprocess
import glob
import time
from geometry_msgs.msg import Quaternion

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
        
def build_classifier(samples=1, nClusters=None, ID='user', used=False):
    if used:
        mdp = pickle.load(open('../data/mdp.pkl'))
        args = {"give_prompt": True,
                "mdp": mdp,
                "id": ID
                } 
        
        progress = myo_state2.Progress(classifier_pkl='../data/state_classifier.pkl', **args)
        return
    
    has_matlab = os.path.exists('/usr/local/MATLAB')
    
    for identifier in ('l', 'u'):
        for i in range(samples):
            data_i, EMG_max_i, EMG_min_i, GYRO_max_i, states_i = preprocess(os.path.join('../../data/work/', str(i)), update_extremes=True, identifier=identifier)
            emg_i = data_i[:, 1:9]
            imu_i = data_i[:, 9:]
            if i == 0:
                emg_demos = emg_i
                imu_demos = imu_i
                emg_data = emg_i
                imu_data = imu_i
                state_labels = states_i
                EMG_MAX = EMG_max_i
                EMG_MIN = EMG_min_i
                GYRO_MAX = GYRO_max_i
                Time_a = np.arange(imu_i.shape[0]).reshape((imu_i.shape[0],1))
                ort_data_a = np.hstack((Time_a, imu_i[:,-4:]))
            else:
                imu_i_a = align_signal(imu_demos/i, imu_i, w=5, has_time=False)
                imu_demos += imu_i_a
                emg_data = np.vstack((emg_data, emg_i))
                imu_data = np.vstack((imu_data, imu_i))
                EMG_MAX += EMG_max_i
                EMG_MIN += EMG_min_i
                GYRO_MAX += GYRO_max_i
                
                state_labels = np.concatenate((state_labels, states_i))
                
                # Used by Matlab function to compute the expected trojectory
                ort_data_a = np.vstack( (ort_data_a, np.hstack((Time_a, imu_i_a[:,-4:]))) )
            
#            # Use original data to train state classifier
#            # aligned data is only used for expected trajectory
#            Time = np.arange(emg_i.shape[0]).reshape((emg_i.shape[0],1))
#            emg_data.append(np.hstack((Time, emg_i)))
#            
#            Time = np.arange(imu_i.shape[0]).reshape((imu_i.shape[0],1))
#            imu_data.append(np.hstack((Time, imu_i)))
            
        
        n = len(imu_demos)
        
        # get average
        if identifier == 'l':
            print "\nProcessing myo on lower arm..."
            emg_demos_l = emg_demos                        
            imu_demos_l = imu_demos/samples
            EMG_MAX_l = EMG_MAX/samples
            EMG_MIN_l = EMG_MIN/samples
            GYRO_MAX_l = GYRO_MAX/samples
            emg_data_l = emg_data
            imu_data_l = imu_data
            ort_data_l = ort_data_a
            print "EMG_MAX_l", EMG_MAX_l
            print "EMG_MIN_l", EMG_MIN_l
            print "GYRO_MAX_l", GYRO_MAX_l
            print "# data points", len(imu_demos_l)
            
        else:
            print "\nProcessing myo on upper arm..."
            emg_demos_u = emg_demos 
            imu_demos_u = imu_demos/samples
            EMG_MAX_u = EMG_MAX/samples
            EMG_MIN_u = EMG_MIN/samples
            GYRO_MAX_u = GYRO_MAX/samples
            emg_data_u = emg_data
            imu_data_u = imu_data
            ort_data_u = ort_data_a
            print "EMG_MAX_u", EMG_MAX_u
            print "EMG_MIN_u", EMG_MIN_u
            print "GYRO_MAX_u", GYRO_MAX_u
            print "# data points", len(imu_demos_u)
            
    N_l = imu_demos_l.shape[0]
    N_u = imu_demos_u.shape[0]
    print N_u
    
    if N_l < N_u:
        emg_demos_u = emg_demos_u[0:N_l, :]
        imu_demos_u = imu_demos_u[0:N_l, :]
        Time = np.arange(N_l).reshape((N_l,1))
    elif N_l > N_u:
        emg_demos_l = emg_demos_l[0:N_u, :]
        imu_demos_l = imu_demos_l[0:N_u, :]
        Time = np.arange(N_u).reshape((N_u,1))
    else:
        Time = np.arange(N_u).reshape((N_u,1))
        
    if has_matlab:
        print "MATLAB FOUND"
        ## use the following if there is Malab under /usr/local
        ## GMM and GMR will be performed
        os.chdir('../matlab')
        #subprocess.call(['matlab', '-nodisplay', '-nojvm', '-nosplash', '-r', 'demo('+str(samples)+');exit'])
        np.savetxt('ort_data_u', ort_data_u, delimiter=',')
        np.savetxt('ort_data_l', ort_data_l, delimiter=',')
        subprocess.call(['matlab', '-nodisplay', '-nojvm', '-nosplash', '-r', 'demo_gmm();exit'])
        os.chdir('../myo_python')
    else:
        print "MATLAB NOT FOUND"
        ## use the following if there is no Matlab
        ## simply use the aligned average as expected trajectory
        np.savetxt('../data/demo_l.dat', imu_demos_l[:, -4:], delimiter=',')
        np.savetxt('../data/demo_u.dat', imu_demos_u[:, -4:], delimiter=',')
             
    emg_demos = np.hstack((emg_demos_l,emg_demos_u))
    emg_cluster = classifier.SignalCluster(emg_demos, n_clusters=8)
    
#    observations = np.hstack((EMG_WEIGHT*emg_demos_l, imu_demos_l, EMG_WEIGHT*emg_demos_u, imu_demos_u))
#    timed_observations = np.hstack((TIME_WEIGHT*Time, EMG_WEIGHT*emg_demos_l, imu_demos_l, EMG_WEIGHT*emg_demos_u, imu_demos_u))
    print emg_data_l.shape, imu_data_l.shape, emg_data_u.shape, imu_data_u.shape
    n_points = min(emg_data_l.shape[0], emg_data_u.shape[0])
    state_labels = state_labels[0:n_points]
    observations = np.hstack((EMG_WEIGHT*emg_data_l[0:n_points, :], imu_data_l[0:n_points, :], EMG_WEIGHT*emg_data_u[0:n_points, :], imu_data_u[0:n_points, :]))

#    if nClusters is None:
#        N = observations.shape[0]
#        low_limit = N/20 # 0.5 states per second
#        high_limit = int(1.5*N/20) # 0.75 states per second
#        scores = {}
#        for n in range(low_limit, high_limit):
#            state_cluster = classifier.SignalCluster(timed_observations, n)
#            score = state_cluster.evaluate(timed_observations, state_cluster.labels)
#            scores[score] = n
#            print '# clusters, score', n, score
#        
#        max_score = max(scores.keys())
#        n_clusters = scores[max_score]
#        state_cluster = classifier.SignalCluster(timed_observations, n_clusters)
#    else:
#        state_cluster = classifier.SignalCluster(timed_observations, nClusters)
        
    plt.figure()
    plt.plot(imu_demos)
    plt.plot(state_labels, '*')
    plt.show(block=True)

    statesData = state_labels[0:len(emg_demos)]
    valid_states = statesData>=0
    actionsData = emg_cluster.labels[valid_states]
    statesData = statesData[valid_states]
    builder = BuildMDP(actionsData=actionsData, statesData=statesData)
    pickle.dump(builder, open('../data/mdp.pkl', 'wb'))
    
    print "path", builder.path
    print "policy", builder.Pi
    with open('../data/state_sequence.dat', 'w') as f:
        for item in builder.path:
            f.write('s'+str(item)+'\n')
    print "expected actions: ", actionsData
    print "expected states: ", statesData
    baseline = evaluate(actionsData, statesData, builder)
    print "baseline performance: ", baseline
    
    
    state_classifier = classifier.SignalClassifier(n_neighbors=5)
    state_classifier.train(observations, state_labels, trainingFile=None)
    pickle.dump((state_classifier, EMG_MAX, EMG_MIN, GYRO_MAX, baseline), open('../data/state_classifier.pkl', 'wb'))
    
    action_classifier = classifier.SignalClassifier(n_neighbors=5)
    action_classifier.train(emg_demos, emg_cluster.labels, trainingFile=None)
    pickle.dump(action_classifier, open('../data/action_classifier.pkl', 'wb'))

def gather_samples_and_build():
    makeProcess = subprocess.Popen(['python', 'makeData2.py', '../../data/work']) 
    makeProcess.wait()

    bagFiles = glob.glob(os.path.join('../../data/work', '*.bag'))
    samples = len(bagFiles)
    print "number of training samples:", samples
    if samples == 0:
        "No data to process!"
        return
    build_classifier(samples=samples)

progress = 0

def signal_handler(msg):
    print "signal received", msg

    global progress

    if msg.data != 1 and progress != 0:
        progress.task = progress.full_task[:]
        progress.start = False
        progress.progress = 0

    if msg.data == 0:
        # There is intentional repetition of this line in both conidition statements rather than
        # at the top of the routine because msg.data is not always 0 or 1
        gather_samples_and_build() 
        demo = myo_state2.MyoPrompt2()
        time.sleep(.5)
        demo.callback(0)
    elif msg.data == 1:
        #gather_samples_and_build() 
        mdp = pickle.load(open('../data/mdp.pkl'))
        args = {"give_prompt": True,
                "mdp": mdp,
                "id": "new patient"
                } 

        progress = myo_state2.Progress(classifier_pkl='../data/state_classifier.pkl', **args)

if __name__ == '__main__':
    import rospy
    from std_msgs.msg import Int32, Empty
    #gather_samples_and_build()
    rospy.init_node('build_classifier')
    rospy.Subscriber('/exercise/mode', Int32, signal_handler)

    MyoDemo2.pub_l = rospy.Publisher('/exercise/l/playback', Quaternion, queue_size=1)
    MyoDemo2.pub_u = rospy.Publisher('/exercise/u/playback', Quaternion, queue_size=1)

    print "Classifier launched. Listening to message..."
    rospy.spin()


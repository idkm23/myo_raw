'''
Created on Mar 25, 2016

@author: ymeng
'''

import cPickle as pickle
import rospy
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import Quaternion
from myo_raw.msg import IMUData, EMGIMU
import numpy as np
import tf
from myo_demo2 import MyoDemo2
import time
import preprocess
from scipy.signal import savgol_filter # need scipy version 0.16
from prompt2 import EMG_WEIGHT, evaluate
import threading
import sys
import math

class MyoPrompt2(MyoDemo2):
    def __init__(self):
        super(MyoPrompt2, self).__init__()
        
    def callback(self, percentage):
        counter = 0
        starting = int(percentage*len(self.quat_l))
        rate = rospy.Rate(10) # 50hz
        for i in range(starting, len(self.quat_l)):
            self.pub_l.publish(self.quat_l[i])
            self.pub_u.publish(self.quat_u[i])
            
            counter += 1
            rate.sleep()
        print counter, "samples pulished."

    def subscriber(self):
        """Do not use subscriber here. 
           The callback function will be called directly without ros message
        """
        pass    

class Progress(object):
    
    def __init__(self, classifier=None, classifier_pkl='../data/state_classifier.pkl', give_prompt=True, **kwargs):
        if classifier:
            self.classifier = classifier
            self.EMG_MAX_l = kwargs.get('EMG_MAX_l', 585)
            self.EMG_MIN_l = kwargs.get('EMG_MIN_l', 0)
            self.GYRO_MAX_l = kwargs.get('GYRO_MAX_l', 500)
            self.EMG_MAX_u = kwargs.get('EMG_MAX_u', 585)
            self.EMG_MIN_u = kwargs.get('EMG_MIN_u', 0)
            self.GYRO_MAX_u = kwargs.get('GYRO_MAX_u', 500)
            self.baseline = kwargs.get('baseline', 0)
        elif classifier_pkl:
            # self.classifier = classifier.SignalClassifier()
            self.classifier, self.EMG_MAX, self.EMG_MIN, self.GYRO_MAX, self.baseline = pickle.load(open(classifier_pkl))
        
        self.give_prompt = give_prompt
        self.baseRot = None
        self.start = False
        self.history = []
        self.user_id = kwargs.get('id', 'new_user')
        self.mdp = kwargs.get('mdp', None)
        
        self.pub = rospy.Publisher('/exercise/progress', Float64, queue_size=10)
        self.pub1 = rospy.Publisher('/exercise/state', String, queue_size=10)
        self.progress = 0.0
        self.count_down = 5 #10 # repeat the state count_down times to be considered entering the state
        self.delay = 0 # time to give prompt if no progress
        self.prompt_now = False
        self.previous = []
        self.getTask()
        #rospy.init_node('prompter')
        if give_prompt:
            self.emgimu_l = None
            self.emgimu_u = None
            try:
                threading.Thread(target=self.activatePrompt).start()
            except:
                print "Could not start thread. ", sys.exc_info()[0]
                sys.exit(1)
#            self.activatePrompt()
#            self.subscribeTrigger()
#            self.starter()
            self.subscribeIMU()
        else:
            self.start = True
            self.subscribeIMU()
        #rospy.spin()
    
    def activatePrompt(self):
        self.prompt = MyoPrompt2()
        #rospy.Subscriber('/exercise/playback_trigger', Empty, self.starter)
        #print "listening to trigger"
        self.starter(None)
            
    def getTask(self):
        with open('../data/state_sequence.dat') as f:
            self.full_task = [x.strip() for x in f]
            self.n_states = len(self.full_task)
            self.task = self.full_task[:] # copy the list
        print self.task
        time.sleep(1)
    
    def callback(self, emgimu):
        if self.emgimu_l is None:
            return
        self.updateUpper(emgimu)
        if self.start and not self.prompt_now:
            self.getProgress(self.emgimu_l, self.emgimu_u)
    
    def updateLower(self, emgimu):
        self.emgimu_l = emgimu
        
    def updateUpper(self, emgimu):
        self.emgimu_u = emgimu
    
    def subscribeIMU(self):
        rospy.Subscriber('/myo/l/emgimu', EMGIMU, self.updateLower, queue_size=2)
        rospy.Subscriber('/myo/u/emgimu', EMGIMU, self.callback, queue_size=2)
    
    def starter(self, msg):
        #self.prompt.callback(0) # publish expected trajectory
        self.start = True
    
#    def subscribeTrigger(self):
#        rospy.Subscriber('/exercise/playback_trigger', Empty, self.starter)
#        print "listening to trigger"
    
    def getProgress(self, emgimu_l, emgimu_u):
        print "Tracking your progress now..."
        
        emg_l = preprocess.process_emg(np.array(emgimu_l.emg), self.EMG_MAX_l, self.EMG_MIN_l)
        acc_l = np.array(emgimu_l.linear_acceleration)
        gyro_l = preprocess.process_gyro(np.array(emgimu_l.angular_velocity), self.GYRO_MAX_l, discrete=False)
        orie_l = np.array(emgimu_l.orientation)
        
        emg_u = preprocess.process_emg(np.array(emgimu_u.emg), self.EMG_MAX_u, self.EMG_MIN_u)
        acc_u = np.array(emgimu_u.linear_acceleration)
        gyro_u = preprocess.process_gyro(np.array(emgimu_u.angular_velocity), self.GYRO_MAX_u, discrete=False)
        orie_u = np.array(emgimu_u.orientation)
        
        signal_array = np.hstack((EMG_WEIGHT*emg_l, acc_l, gyro_l, orie_l, EMG_WEIGHT*emg_u, acc_u, gyro_u, orie_u))
        self.history.append(signal_array)
        if len(self.previous)>10: # 0.2 seconds
            self.previous.pop(0)
        self.previous.append(signal_array)
        current_signal = np.mean(self.previous, axis=0)

        state = 's'+str(int(self.classifier.predict(current_signal)[0]))
        self.pub1.publish(state)
        self.pub.publish(self.progress)
        if self.task == []:
            print "Task completed!"
            self.pub.publish(1.0)
            self.task = self.full_task[:]
            self.start = False
            self.progress = 0
            self.evaluate_pfmce()
            
            #sys.exit()
        if state == self.task[0]:
            self.count_down -= 1
            if self.count_down == 0:
                # print "current state", state, state_map[state]
                self.task.pop(0)
                self.progress = 1 - 1.0*len(self.task)/(self.n_states-1)
                self.pub.publish(self.progress)
                self.count_down = 10
                self.delay = 0
        elif self.give_prompt:
            # no transition
            self.delay += 1
            if len(self.task)>0 and self.delay>500:
                #self.start = False
                self.prompt_now = True
                print "prompt started"
                print self.prompt_now
                self.prompt.callback(self.progress)
                print "prompt ended"
                self.delay = 0
                self.prompt_now = False
                
    def evaluate_pfmce(self):
        print "Evaluating performance...."
        if not self.mdp:
            print "No MDP found"
            self.history = []
            return
        action_classifier = pickle.load(open('../data/action_classifier.pkl'))
        history = np.array(self.history)
        history = history[25:-6:5, :] # downsampling, cut the first half second
        history = savgol_filter(history, 31, 3, axis=0) # smoothing
        actions = []
        states = []
        for signal in history:
            emg_l = signal[0:8]/EMG_WEIGHT
            emg_u = signal[18:26]/EMG_WEIGHT
            emg = np.hstack((emg_l,emg_u))
            actions.append(int(action_classifier.predict(emg)[0]))
            states.append(int(self.classifier.predict(signal)[0]))
        print actions
        print states
        result = evaluate(actions, states, self.mdp)
        print result
        score = 100*math.exp((result-self.baseline)/200)
        print "Performance score =", int(score)
        np.savetxt('user_data/'+self.user_id, history, delimiter=',')
        self.history = []     


if __name__ == '__main__':
    progress = Progress(give_prompt=True)

    
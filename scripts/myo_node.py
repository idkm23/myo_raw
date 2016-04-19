#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, Pose, PoseStamped, Point
from myo_raw.msg import Gesture, EMGData, MyoArm, IMUData, EMGIMU
from myo.myo import Myo, NNClassifier
from myo.myo_raw import MyoRaw

from time import sleep
from threading import Thread

import tf
import ctypes
import rospkg
import os
from math import pi

# Scale values for unpacking IMU data
# Accelerometer data. In units of g. Range of + -16. Values are multiplied by MYOHW_ACCELEROMETER_SCALE.
# Gyroscope data. In units of deg/s. Range of + -2000. Values are multiplied by MYOHW_GYROSCOPE_SCALE.
MYOHW_ORIENTATION_SCALE = 16384.0 
MYOHW_ACCELEROMETER_SCALE = 2048.0
MYOHW_GYROSCOPE_SCALE = 16.0

UPPER_ARM_DEVICE_NAME = "Winnie's Myo"

class MyoNode(object):
    """A ros wrapper for myo_raw"""
    def __init__(self):

        self.m = Myo(NNClassifier())
        self.connect(self.m)

        if self.m.device_name == UPPER_ARM_DEVICE_NAME:
            self.identifier  = 'u'
        else:
            self.identifier  = 'l'

        #might be necessary for some classification?
        #traning_data_directory = rospy.get_param('~training_data_dir', None)

        self.pub_imu     = rospy.Publisher('/myo/' + self.identifier + '/imu',     IMUData,     queue_size=10)
        self.pub_emg     = rospy.Publisher('/myo/' + self.identifier + '/emg',     EMGData,     queue_size=10)
        self.pub_pose    = rospy.Publisher('/myo/' + self.identifier + '/gesture', Gesture,     queue_size=10)
        self.pub_myo_arm = rospy.Publisher('/myo/' + self.identifier + '/arm',     MyoArm,      queue_size=10, latch=True)
        self.pub_ort     = rospy.Publisher('/myo/' + self.identifier + '/ort',     Quaternion,  queue_size=10)
        self.pub_emgimu  = rospy.Publisher('/myo/' + self.identifier + '/emgimu',  EMGIMU,      queue_size=10)
        
        self.downsampler = 0
        self.baseRot = None 
        self.count = 0 

        #might be necessary for some classification?
        #os.chdir(rospkg.RosPack().get_path('myo_raw')+'/training_data' if traning_data_directory == None else traning_data_directory)

    def connect(self, m):
        m.connect()

        m.vibrate(2)
        m.add_emg_handler(self.__on_emg)
        m.add_imu_handler(self.__on_imu)
        # m.add_pose_handler(self.__on_pose)
        m.add_raw_pose_handler(self.__on_raw_pose)
        m.add_arm_handler(self.__on_arm)

    def disconnect(self):
        self.m.disconnect()
    
    # builtin pose subscriber
    # def __on_pose(self, p):
    #     self.pub_pose.publish(is_builtin=True, pose_number=p, confidence=0.0)

    def __on_arm(self, arm, xdir):
        self.pub_myo_arm.publish(arm=arm, xdir=xdir)
    
    def __on_emg(self, emg, moving):
        self.pub_emg.publish(emg_data=emg, moving=moving)
        self.emg = emg

    def __on_raw_pose(self, p):
        self.pub_pose.publish(is_builtin=False, pose_number=p, confidence=0.0)

    def __on_imu(self, quat, acc, gyro):
        # need to switch the yaw and the roll for some reason
        euler = tf.transformations.euler_from_quaternion((quat[0], quat[1], quat[2], quat[3])) # roll, pitch, yaw
        
        if self.baseRot == None:
            self.count += 1;

            # This subtraction '(60 * pi / 180.0)' is here to create an initial offset which matches a hand's natural initial offset
            if self.count > 360:
                self.baseRot = euler[0]
            return

        # need to switch the yaw and the roll for some reason   2    1     0
        rotated_quat = tf.transformations.quaternion_from_euler(-euler[2], euler[1], -euler[0] + self.baseRot)


        self.pub_ort.publish(Quaternion(x=rotated_quat[0], 
                                        y=rotated_quat[1], 
                                        z=rotated_quat[2], 
                                        w=rotated_quat[3]))

        self.pub_imu.publish(header=Header(frame_id=rospy.get_param('frame_id', 'map'), stamp=rospy.get_param('stamp', None)),
                               angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                               linear_acceleration=Vector3(x=acc[0]/MYOHW_ACCELEROMETER_SCALE, y=acc[1]/MYOHW_ACCELEROMETER_SCALE, z=acc[2]/MYOHW_ACCELEROMETER_SCALE),
                               orientation=Quaternion(x=rotated_quat[0], y=rotated_quat[1], z=rotated_quat[2], w=rotated_quat[3])
                               )

        if self.emg:
            self.pub_emgimu.publish(header=Header(frame_id=rospy.get_param('frame_id', 'map'), stamp=rospy.get_param('stamp', None)),
                               emg=self.emg,
                               angular_velocity=gyro,
                               linear_acceleration=[acc[0]/MYOHW_ACCELEROMETER_SCALE, acc[1]/MYOHW_ACCELEROMETER_SCALE, acc[2]/MYOHW_ACCELEROMETER_SCALE],
                               orientation=[rotated_quat[0], rotated_quat[1], rotated_quat[2], rotated_quat[3]]
                               )

    def run(self): # note this function is EXTREAMELY time sensitive... delay will cause a disconnect of the myo
        try:    
            while not rospy.is_shutdown():
                self.m.run()
        except rospy.ROSInterruptException or KeyboardInterrupt:
            self.m.disconnect()

        print "myo node shutting down"

class MyThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.m = MyoNode()

    def run(self):
        self.m.run()

if __name__ == '__main__':
    rospy.init_node('myo_node', anonymous=True)

    t1 = None
    t2 = None

    while t1 == None or t2 == None or t1.m == None or t2.m == None or t1.m.m.initialized == False or t2.m.m.initialized == False:
        if t1 == None or t1.m == None or t1.m.m.initialized == False:
            if t1 != None and t1.m != None:
                t1.m.unblock_tty()

            t1 = MyThread()
            t1.daemon = True
            t1.start()

            sleep(3)

        if t2 == None or t2.m == None or t2.m.m.initialized == False:
            if t2 != None and t2.m != None:
                t2.m.unblock_tty()

            t2 = MyThread()
            t2.daemon = True
            t2.start()

            sleep(4)

    print "\nEnter '1' at any time to reset the coordinate which is relative to north"



    try:
        while not rospy.is_shutdown():
            char = raw_input('')
            if char == '1':
                if t1 != None and t1.m != None:
                    t1.m.baseRot = None
                if t2 != None and t2.m != None:
                    t2.m.baseRot = None
    except rospy.ROSInterruptException or KeyboardInterrupt or EOFError:    
        t1.join()
        t2.join()

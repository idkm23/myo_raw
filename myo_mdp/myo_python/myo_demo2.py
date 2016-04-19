'''
Created on Mar 24, 2016

@author: ymeng
'''
#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from numpy import genfromtxt
from geometry_msgs.msg import Quaternion
import tf


class MyoDemo2(object):
    def __init__(self, use_euler=False):

        self.pub_l = rospy.Publisher('/exercise/l/playback', Quaternion, queue_size=20)
        self.pub_u = rospy.Publisher('/exercise/u/playback', Quaternion, queue_size=20)
    
    
        orientation_l = genfromtxt('../data/demo_l.dat', delimiter=',')
        orientation_u = genfromtxt('../data/demo_u.dat', delimiter=',')
        
        self.quat_l = []
        self.quat_u = []
        if use_euler:
            for euler_l in orientation_l:
                rotated_quat_l = tf.transformations.quaternion_from_euler(euler_l[2], euler_l[1], euler_l[0])
                self.quat_l.append(Quaternion(x=rotated_quat_l[0], 
                                  y=rotated_quat_l[1], 
                                  z=rotated_quat_l[2], 
                                  w=rotated_quat_l[3]))
                
            for euler_u in orientation_u:
                rotated_quat_u = tf.transformations.quaternion_from_euler(euler_u[2], euler_u[1], euler_u[0])
                self.quat_u.append(Quaternion(x=rotated_quat_u[0], 
                                  y=rotated_quat_u[1], 
                                  z=rotated_quat_u[2], 
                                  w=rotated_quat_u[3]))            
        else:
            for q in orientation_l:
                self.quat_l.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

            for q in orientation_u:
                self.quat_u.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        
        self.quat_l.append(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
        self.quat_u.append(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
        
        

    def callback(self, message):
        counter = 0
        rate = rospy.Rate(50) # 50hz
        for q in self.quat:
            self.pub.publish(q)
            counter += 1
            rate.sleep()
        print counter, "coordinates pulished."
        
    def subscriber(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/exercise/playback_trigger', Empty, self.callback)
        rospy.spin()
    
if __name__ == '__main__':
    demo = MyoDemo2()
    print demo.quat_l
    print demo.quat_u

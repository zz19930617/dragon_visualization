#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import time

def talker():
    hip_data = [0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
    knee_data = [1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.35, 1.4, 1.45, 1.5]
    pub = rospy.Publisher('/debug', Float64MultiArray, queue_size = 10)
    rospy.init_node('debug_list', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = [0,0,0, 0, 0, 0, 0, 0]
        for i in range(10):
            msg.data[0] = hip_data[i]
            msg.data[1] = knee_data[i]
            pub.publish(msg)
            time.sleep(0.3)
            rate.sleep()
            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
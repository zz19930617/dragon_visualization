#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import time

def talker():
    hip_data = [0.4,0.44,0.48,0.52,0.56,0.57,0.59,0.62,0.64,0.66]
    knee_data = [1.2,1.22,1.24,1.26,1.28,1.3,1.32,1.34,1.36,1.38]
    pub = rospy.Publisher('/debug', Float64MultiArray, queue_size = 10)
    rospy.init_node('debug_list', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = [0,0,0, 0, 0, 0, 0, 0]
        for i in range(10):
            msg.data[4] = hip_data[i]
            msg.data[5] = knee_data[i]
            pub.publish(msg)
            time.sleep(1)
            rate.sleep()
            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
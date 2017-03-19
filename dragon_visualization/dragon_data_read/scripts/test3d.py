#!/usr/bin/env python
# license removed for brevitys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

global index
global z_index
z_index = 0
index =0
def talker():
    pub = rospy.Publisher('/dragon/test3d', Float64MultiArray, queue_size=10)
    rospy.init_node('talker3d', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = [0,0,0,0,0,0,0,0]
        theta = np.linspace(-4 * np.pi, 4 * np.pi, 10)
	global z_index
        z = np.linspace(-2, 2, 10)+z_index
	z_index = z_index + 0.2
	global index
        r = z**2 + index
        x = r * np.sin(theta)
        y = r * np.cos(theta)
	if max(x) >1000:
	    z_index = 0
	    index = 0
	    z = np.linspace(-2, 2, 10)+z_index
	    r = z**2 + index
	    x = r * np.sin(theta)
	    y = r * np.cos(theta)		    
        for i in range(len(x)):
	    msg.data[0] = x[i]
	    msg.data[1] = y[i]
	    msg.data[2] = z[i]
	    pub.publish(msg)
	index = index +1
        rate.sleep()
if __name__ == '__main__':
        try:
        	talker()
        except rospy.ROSInterruptException:
        	pass
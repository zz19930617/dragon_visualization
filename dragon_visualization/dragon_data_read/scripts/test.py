#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('/dragon/joint_angle_command', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/joint_states', JointState , queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg2 = JointState()
        msg.data = [0,0,0,0,0,0,0,0,0,0,0,0]
        msg2.position = [0,0,0,0,0,0,0,0,0,0,0,0]
        x = 1000
        while x:
            x = x -1
            for y in range(12):
                msg.data[y] = msg.data[y] +y+1
                msg2.position[y] = msg2.position[y]+5*y+3
            pub.publish(msg)
            pub2.publish(msg2)
            rate.sleep()
if __name__ == '__main__':
        try:
        	talker()
        except rospy.ROSInterruptException:
        	pass
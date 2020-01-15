#!/usr/bin/env python
import time
import rospy
import message_filters
from roboclaw_wrapper import MotorControllers
from sensor_msgs.msg import Joy


motorcontrollers0 = MotorControllers(0,128)
motorcontrollers1 = MotorControllers(1,128)

def joy_callback(data):
	speedright = -data.axes[1]*100
	speedleft = data.axes[4]*100
	for i in range(2):
		motorcontrollers0.sendSignedDutyAccel(i,speedleft)
		motorcontrollers1.sendSignedDutyAccel(i,speedright)



if __name__ == '__main__':
	rospy.init_node('rover')
	rospy.loginfo("Starting the rover node")
	global pub
	joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)
	rate = rospy.Rate(10)
	rate.sleep()
	rospy.spin()



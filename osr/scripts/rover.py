#!/usr/bin/env python
import time
import rospy
import message_filters
from roboclaw_wrapper import MotorControllers
from rover_udes.msg import Command


motorcontrollers0 = MotorControllers(0,128)
motorcontrollers1 = MotorControllers(1,128)


def cmd_mux_callback(data):
	speedright = -data.right
	speedleft = data.left
	for i in range(2):
		motorcontrollers0.sendSignedDutyAccel(i, speedleft)
		motorcontrollers1.sendSignedDutyAccel(i, speedright)


if __name__ == '__main__':
	rospy.init_node('rover')
	rospy.loginfo("Starting the rover node")
	cmd_mux_sub = rospy.Subscriber("/mux_cmd", Command, cmd_mux_callback)
	rate = rospy.Rate(10)
	rate.sleep()
	rospy.spin()



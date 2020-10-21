#!/usr/bin/env python

# Description: This node takes as input the velocity target of the wheels and converts it to motor values. 
#              It also publishes the encoder values for both sides
# Author: Jeremie Bourque - jeremie.bourque@usherbrooke.ca

import time
import rospy
import threading
from roboclaw_wrapper import MotorControllers
from differential_drive.msg import VelTarget, Encoders

#TODO: If no message is received, velocity should go back to 0
#TODO: If node is killed, velocity should be 0
#TODO: Take into consideration the max speed of the robot
#FIXME: Sometimes the the motor commands are delayed, causing them to move even if I stopped touching the remote. Get rid of cmd buffer.

class MotorVelocityController:
    def __init__(self):
        self.mutex = threading.Lock()
        self.left_motor_controller = MotorControllers(1,128)  # Left
        self.right_motor_controller = MotorControllers(0,128)  # Right
        self.sub_wheel_vel_target = rospy.Subscriber("/wheel_vel_target", VelTarget, self.motorCmdCB)
        self.pub_wheel_enc = rospy.Publisher("/wheel_enc", Encoders, queue_size=1)
        self.upper_limit = 100
        self.lower_limit = -100
        self.maximum_speed = 0.5  # m/s

    def motorCmdCB(self, msg):
        for i in range(2):
            self.mutex.acquire()
            left_cmd = self.limitValue(-msg.left_wheel_vel_target / self.maximum_speed * self.upper_limit)
            right_cmd = self.limitValue(msg.right_wheel_vel_target / self.maximum_speed * self.upper_limit)
            #print("left_cmd=%f  right_cmd=%f" % (left_cmd, right_cmd))
            self.left_motor_controller.sendSignedDutyAccel(i, left_cmd)
            self.right_motor_controller.sendSignedDutyAccel(i, right_cmd)
            self.mutex.release()
    
    def limitValue(self, value):
        if value > self.upper_limit:
            value = self.upper_limit
        elif value < self.lower_limit:
            value = self.lower_limit
        return value


    def loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.mutex.acquire()
            left_enc = self.left_motor_controller.getDriveEnc()
            right_enc = self.right_motor_controller.getDriveEnc()
            self.mutex.release()
            wheel_enc = Encoders()
            wheel_enc.left_encoder = -(int((left_enc[0] + left_enc[1])/2))
            wheel_enc.right_encoder = (int((right_enc[0] + right_enc[1])/2))
            self.pub_wheel_enc.publish(wheel_enc)
        rate.sleep()
    

if __name__ == '__main__':
    rospy.init_node('motor_velocity_controller')
    rospy.loginfo("Starting the motor_velocity_controller node")
    motor_velocity_controller = MotorVelocityController()
    motor_velocity_controller.loop()
    



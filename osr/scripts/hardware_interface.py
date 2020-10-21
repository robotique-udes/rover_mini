#!/usr/bin/env python

# Description: This node takes as input the velocity target of the wheels and converts it to motor values. 
#              It also publishes the encoder values for both sides
# Author: Jeremie Bourque - jeremie.bourque@usherbrooke.ca

import time
import rospy
import threading
from roboclaw_wrapper import MotorControllers
from differential_drive.msg import VelTarget, Encoders

#TODO: If node is killed, velocity should be 0
#TODO: Take into consideration the max speed of the robot

class MotorVelocityController:
    def __init__(self):
        self.mutex = threading.Lock()
        self.left_motor_controller = MotorControllers(1,128)  # Left
        self.right_motor_controller = MotorControllers(0,128)  # Right
        self.sub_wheel_vel_target = rospy.Subscriber("/wheel_vel_target", VelTarget, self.motorCmdCB, queue_size=1)
        self.pub_wheel_enc = rospy.Publisher("/wheel_enc", Encoders, queue_size=1)
        self.upper_limit = 100
        self.lower_limit = -100
        self.maximum_speed = 0.5  # m/s
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.ticks = 0

    def motorCmdCB(self, msg):
        self.mutex.acquire()
        for i in range(2):
            left_cmd = self.limitValue(-msg.left_wheel_vel_target / self.maximum_speed * self.upper_limit)
            right_cmd = self.limitValue(msg.right_wheel_vel_target / self.maximum_speed * self.upper_limit)
            #print("left_cmd=%f  right_cmd=%f" % (left_cmd, right_cmd))
            self.left_motor_controller.sendSignedDutyAccel(i, left_cmd)
            self.right_motor_controller.sendSignedDutyAccel(i, right_cmd)
        self.mutex.release()
        self.ticks = 0
    
    def limitValue(self, value):
        if value > self.upper_limit:
            value = self.upper_limit
        elif value < self.lower_limit:
            value = self.lower_limit
        return value


    def loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.ticks > self.timeout_ticks:
                # Stop motors
                self.mutex.acquire()
                for i in range(2):
                    self.left_motor_controller.sendSignedDutyAccel(i, 0)
                    self.right_motor_controller.sendSignedDutyAccel(i, 0)
                self.mutex.release()
            self.mutex.acquire()
            left_enc = self.left_motor_controller.getDriveEnc()
            right_enc = self.right_motor_controller.getDriveEnc()
            self.mutex.release()
            wheel_enc = Encoders()
            wheel_enc.left_encoder = -(int((left_enc[0] + left_enc[1])/2))
            wheel_enc.right_encoder = (int((right_enc[0] + right_enc[1])/2))
            self.pub_wheel_enc.publish(wheel_enc)
            self.ticks += 1
        rate.sleep()
    

if __name__ == '__main__':
    rospy.init_node('motor_velocity_controller')
    rospy.loginfo("Starting the motor_velocity_controller node")
    motor_velocity_controller = MotorVelocityController()
    motor_velocity_controller.loop()
    



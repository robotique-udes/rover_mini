#!/usr/bin/env python

# Description: This node takes as input the velocity target of the wheels and converts it to motor values. 
#              It also publishes the encoder values for both sides
# Author: Jeremie Bourque - jeremie.bourque@usherbrooke.ca

import time
import rospy
import threading
from copy import deepcopy
from roboclaw_wrapper import MotorControllers
from differential_drive.msg import VelocityTargets, Encoders

class MotorVelocityController:
    def __init__(self):
        self.mutex = threading.Lock()
        self.left_motor_controller = MotorControllers(1,128)  # Left
        self.right_motor_controller = MotorControllers(0,128)  # Right
        self.sub_wheel_vel_target = rospy.Subscriber("/wheel_vel_target", VelocityTargets, self.velocityTargetsCB, queue_size=1)
        self.pub_wheel_enc = rospy.Publisher("/wheel_enc", Encoders, queue_size=1)
        self.upper_limit = 100
        self.lower_limit = -100
        self.maximum_speed = 0.5  # m/s
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.ticks = 0
        self.encoder_min = rospy.get_param("~encoder_min", -32768)
        self.encoder_max = rospy.get_param("~encoder_max", 32768)
        self.encoder_low_wrap = rospy.get_param("~wheel_low_wrap", (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param("~wheel_high_wrap", (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)
        self.ticks_per_meter= float(rospy.get_param("~ticks_per_meter", 50))

        self.wheel_enc = Encoders()  # Wheel encoders with limits
        self.prev_wheel_enc = Encoders()  # Previous wheel encoders with limits
        self.wheel_enc_wrap = Encoders()  # Wheel encoders with wrap around
        self.prev_wheel_enc_wrap = Encoders()  # Previous wheel encoders with wrap around
        self.prev_wheel_enc_wrap.left_encoder = None 
        self.prev_wheel_enc_wrap.right_encoder = None 

    def velocityTargetsCB(self, msg):
        #TODO: Implement PID
        self.mutex.acquire()
        for i in range(2):
            left_cmd = self.limitValue(-msg.left_wheel_vel_target / self.maximum_speed * self.upper_limit)
            right_cmd = self.limitValue(msg.right_wheel_vel_target / self.maximum_speed * self.upper_limit)
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

    def handleWrapAround(self):
        # Wrap around for left encoder
        if (self.wheel_enc.left_encoder < self.encoder_low_wrap and self.prev_wheel_enc.left_encoder > self.encoder_high_wrap):
            self.wheel_enc.left_encoder_multiplier += 1
        elif (self.wheel_enc.left_encoder > self.encoder_high_wrap and self.prev_wheel_enc.left_encoder < self.encoder_low_wrap):
            self.wheel_enc.left_encoder_multiplier.left_nb_of_wraps -= 1
        # Wrap around for right encoder
        if (self.wheel_enc.right_encoder < self.encoder_low_wrap and self.prev_wheel_enc.right_encoder > self.encoder_high_wrap):
            self.wheel_enc.right_encoder_multiplier += 1
        elif (self.wheel_enc.right_encoder > self.encoder_high_wrap and self.prev_wheel_enc.right_encoder < self.encoder_low_wrap):
            self.wheel_enc.right_encoder_multiplier -= 1
        # Calculate encoder values with wrap
        self.wheel_enc_wrap.left_encoder = self.wheel_enc.left_encoder + self.wheel_enc.left_encoder_multiplier * (self.encoder_max - self.encoder_min)
        self.wheel_enc_wrap.right_encoder = self.wheel_enc.right_encoder + self.wheel_enc.right_encoder_multiplier * (self.encoder_max - self.encoder_min)
        self.prev_wheel_enc = deepcopy(self.wheel_enc)

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
            self.handleWrapAround()
            self.pub_wheel_enc.publish(wheel_enc)
            self.ticks += 1
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rospy.init_node('motor_velocity_controller')
        rospy.loginfo("Starting the motor_velocity_controller node")
        motor_velocity_controller = MotorVelocityController()
        motor_velocity_controller.loop()
    except KeyboardInterrupt:
        # Stop motors
        motor_velocity_controller.mutex.acquire()
        for i in range(2):
            motor_velocity_controller.left_motor_controller.sendSignedDutyAccel(i, 0)
            motor_velocity_controller.right_motor_controller.sendSignedDutyAccel(i, 0)
        motor_velocity_controller.mutex.release()
        raise
    



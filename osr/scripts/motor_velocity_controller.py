#!/usr/bin/env python
import time
import rospy
import threading
from roboclaw_wrapper import MotorControllers
from std_msgs.msg import Float32, Int32


left_motor_controller = MotorControllers(0,128)  # Left
right_motor_controller = MotorControllers(1,128)  # Right


def rmotor_cmd_callback(msg):
    for i in range(2):
	mutex.acquire()
        right_motor_controller.sendSignedDutyAccel(i, msg.data)
	mutex.release()

def lmotor_cmd_callback(msg):
    for i in range(2):
        mutex.acquire()
        left_motor_controller.sendSignedDutyAccel(i, -msg.data)
        mutex.release()
    

if __name__ == '__main__':
    mutex = threading.Lock()
    rospy.init_node('motor_velocity_controller')
    rospy.loginfo("Starting the motor_velocity_controller node")
    sub_rmotor_cmd = rospy.Subscriber("/rmotor_cmd", Float32, rmotor_cmd_callback)
    sub_lmotor_cmd = rospy.Subscriber("/lmotor_cmd", Float32, lmotor_cmd_callback)
    pub_rwheel_enc = rospy.Publisher("/rwheel", Int32, queue_size=10)
    pub_lwheel_enc = rospy.Publisher("/lwheel", Int32, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        mutex.acquire()
        left_enc = left_motor_controller.getDriveEnc()
        right_enc = right_motor_controller.getDriveEnc()
        mutex.release()
        avg_left_enc = int((left_enc[0] + left_enc[1])/2)
        avg_right_enc = -(int((right_enc[0] + right_enc[1])/2))
        pub_lwheel_enc.publish(avg_left_enc)
        pub_rwheel_enc.publish(avg_right_enc)
	rate.sleep()



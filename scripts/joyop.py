#!/usr/bin/env python

'''
ackermann_drive_joyop.py:
    A ros joystick teleoperation script for ackermann steering based robots
'''

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
import sys


class AckermannDriveJoyop:

    def __init__(self, args):
        if len(args) == 1 or len(args) == 2:
            self.max_speed = float(args[0])
            self.max_steering_angle = float(args[len(args)-1])
            cmd_topic = 'ackermann_cmd'
        elif len(args) == 3:
            self.max_speed = float(args[0])
            self.max_steering_angle = float(args[1])
            cmd_topic = '/' + args[2]
        else:
            self.max_speed = 1.0
            self.max_steering_angle = 1.0
            cmd_topic = 'ackermann_cmd'

        self.speed = 0.0
        self.steering_angle = 0.0
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.drive_pub = rospy.Publisher(cmd_topic, AckermannDriveStamped,
                                         queue_size=1)
	self.armed = False
	self.forward_active = False
	self.reverse_active = False

    def joy_callback(self, joy_msg):
		self.forward_active = joy_msg.axes[4] < 0.99
		self.reverse_active = joy_msg.axes[5] < 0.99
		if(joy_msg.axes[4] < -0.5 and joy_msg.axes[5] < -0.5):
			self.armed = True
	        self.print_state()
		if not self.armed:
			return
		if (self.forward_active and self.reverse_active):
			# Brake
			self.speed = 0.0
		elif (self.reverse_active):
			# Reverse
			self.speed = -(joy_msg.axes[5] - 1.0)/2.0 * self.max_speed
		elif (self.forward_active):
			# Forward
			self.speed = (joy_msg.axes[4] - 1.0)/2.0 * self.max_speed
		else:
			# Coast
			self.speed = float('nan')  # TODO : use nan

		# Steering
		if abs(joy_msg.axes[0]) > 0.1:
	        	self.steering_angle = - joy_msg.axes[0] * self.max_steering_angle
		else:
			self.steering_angle = 0.0

		# Publish control commands
		ackermann_cmd_msg = AckermannDriveStamped()
		ackermann_cmd_msg.drive.speed = self.speed
		ackermann_cmd_msg.drive.steering_angle = self.steering_angle
		self.drive_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mArmed: \033[32;1m%s, '
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.armed, self.speed if not math.isnan(self.speed) else 0.0, self.steering_angle)

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.drive_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])
    rospy.spin()

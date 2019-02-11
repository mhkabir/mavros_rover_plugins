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
            self.max_acceleration = float(args[0])
            self.max_steering_angle = float(args[len(args) - 1])
        else:
            self.max_acceleration = 1.0     # m/s^2
            self.max_steering_angle = 0.5   # rad

        self.acceleration = 0.0
        self.steering_angle = 0.0
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.manual_pub = rospy.Publisher('/ecu_interface/control/ackermann_cmd_manual', AckermannDriveStamped,
                                         queue_size=1)
        self.auto_pub = rospy.Publisher('/ecu_interface/control/ackermann_cmd_auto', AckermannDriveStamped,
                                         queue_size=1)
        # If true, send acceleration/velocity setpoints from joystick when autonomous control button is pressed,
        # instead of direct normalized torque commands.
        # If false, joystick setpoints are stopped completely when autonomous
        # control button is pressed.
        self.manual_metric_control = False
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
            self.acceleration = 0.0
        elif (self.reverse_active):
            # Reverse
            self.acceleration = (joy_msg.axes[5] - 1.0) / 2.0
        elif (self.forward_active):
            # Forward
            self.acceleration = -(joy_msg.axes[4] - 1.0) / 2.0
        else:
            # Coast
            self.acceleration = float('nan')

        # Steering
        if abs(joy_msg.axes[0]) > 0.1:
                self.steering_angle = - joy_msg.axes[0]
        else:
            self.steering_angle = 0.0

        self.print_state()

        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp = rospy.Time.now()
        ackermann_cmd_msg.drive.speed = float('nan')
        ackermann_cmd_msg.drive.jerk = float('nan')
        if joy_msg.buttons[0] == 0:
            # Publish normalized manual setpoints if autonomous control button isn't held down
            ackermann_cmd_msg.drive.acceleration = self.acceleration
            ackermann_cmd_msg.drive.steering_angle = self.steering_angle
            self.manual_pub.publish(ackermann_cmd_msg)
        elif self.manual_metric_control:
            # Publish metric manual setpoints if enabled
            ackermann_cmd_msg.drive.acceleration = self.acceleration * self.max_acceleration
            ackermann_cmd_msg.drive.steering_angle = self.steering_angle * self.max_steering_angle
            self.auto_pub.publish(ackermann_cmd_msg)


    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mArmed: \033[32;1m%s, '
                      '\033[34;1mAcceleration: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.armed, self.acceleration if not math.isnan(self.acceleration) else 0.0, self.steering_angle)

    def finalize(self):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.acceleration = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.manual_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])
rospy.spin()

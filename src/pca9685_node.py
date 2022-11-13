#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2022 Eric Wang
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
import sys
import time
import Jetson.GPIO as GPIO
from pca9685_driver import Device

import rospy
from geometry_msgs.msg import Twist
from ros_pca9685_drive.msg import Servos

class PCA9685DriveException(Exception):
    pass

class Servo_Config():
    def __init__(self, name, params):
        self.name = name
        for key in ['channel', 'center', 'direction', 'range']:
            if not key in params:
                raise PCA9685DriveException("Missing configuration for 'drive_servos/{}/{}'.".format(name, key))
        self.channel = int(params['channel'])
        self.direction = int(params['direction'])
        self.center = int(params['center'])
        self.range = int(params['range'])
        rospy.loginfo("PCA9685 servo: %s, Channel: %d, Direction: %d, Center: %d, Range: %d", self.name, self.channel, self.direction, self.center, self.range)

class PCA9685_Drive_Node():
    def __init__(self):
        if not rospy.has_param("~drive_servos"):
            raise PCA9685DriveException("No configurations found for the 'drive_servos' parameter.")

        # Get parameters
        params = rospy.get_param("~drive_servos")
        self.topic_drive = params.get('topic_drive', '/cmd_vel')
        self.topic_absolute = params.get('topic_absolute', '/servos_raw')
        self.i2c_bus = int(params.get('i2c_bus', 1))
        self.i2c_baseAddr = int(params.get('i2c_base_address', 0x4000))
        self.pwm_frequency = int(params.get('pwm_frequency', 50))
        rospy.loginfo("Subscribe on topic_drive: %s", self.topic_drive)
        rospy.loginfo("Subscribe on topic_absolute: %s", self.topic_absolute)
        rospy.loginfo("PCA9685 PWM frequency: %d", self.pwm_frequency)

        # Get servo configurations
        self.servos_config = {}
        for key in ['steering', 'throttle']:
            if not key in params:
                raise PCA9685DriveException("Missing configuration for 'drive_servos/{}'.".format(key))
            self.servos_config[key] = Servo_Config(key, params[key])

        # Initialize the I2C PCA9685 device driver and reset servos to the center position
        GPIO.setmode(GPIO.BOARD)
        self.device = Device(self.i2c_baseAddr, self.i2c_bus)
        self.device.wake()
        self.device.set_pwm_frequency(self.pwm_frequency)
        self.forwarded = False
        self.setPWM(self.servos_config['steering'].channel, self.servos_config['steering'].center)
        self.setPWM(self.servos_config['throttle'].channel, self.servos_config['throttle'].center)

        # Subscribe to the topics 
        rospy.Subscriber(self.topic_drive, Twist, self.drive_callback)
        rospy.Subscriber(self.topic_absolute, Servos, self.servos_callback)

    def drive_callback(self, twist_data):
        # Set throttling
        self.drive_SetPWM('throttle', twist_data.linear.x)

        # Set sterring
        self.drive_SetPWM('steering', twist_data.angular.z)

    def servos_callback(self, servos_data):
        vl = len(servos_data.value)
        for i in range(len(servos_data.servos)):
            servo = servos_data.servos[i]
            if not servo in self.servos_config:
                rospy.logerr("Can not find configuration for servo '%s', skipped.", servo)
                continue
            servoConfig = self.servos_config[servo]
            if i >= vl:
                rospy.logerr("Lack of value for servo '%s', skip.", servo)
                continue
            pwm = int(servos_data.value[i])
            p = self.setPWM(servoConfig.channel, pwm)
            rospy.loginfo("Directly set PWM value on servo %s: %d", servo, p)

    def drive_SetPWM(self, servo, value):
        servoConfig = self.servos_config[servo]
        v = max(min(value, 1.0), -1.0)
        pwm = int(servoConfig.center + servoConfig.direction * v * servoConfig.range / 2.0)
        p = self.setPWM(servoConfig.channel, pwm)
        rospy.loginfo("Servo %s: value %.3f, PWM %d, forwarded: %s", servo, value, p, self.forwarded)
    
    def setPWM(self, channel, pwm):
        try:
            p = max(min(pwm, 4095), 0)

            # ESC requires resetting the center PWM value before reverse
            servo_throttle = self.servos_config['throttle']
            if channel == servo_throttle.channel:
                if p < servo_throttle.center:
                    if self.forwarded:
                        rospy.loginfo("Rese PWM value to reverse.")
                        self.device.set_pwm(channel, int(servo_throttle.center - servo_throttle.range/8))
                        time.sleep(0.05)
                        self.device.set_pwm(channel, servo_throttle.center)
                        time.sleep(0.05)
                    self.forwarded = False
                elif p > servo_throttle.center:
                    self.forwarded = True

            self.device.set_pwm(channel, p)
            return p
        except Exception as e:
            rospy.logerr("Error trying to set PWM value %d on the PCA9685 board: %s", p, e)

if __name__ == "__main__":
    try:
        rospy.init_node("pca9685_drive")
        node = PCA9685_Drive_Node()
        rospy.spin()
    except PCA9685DriveException as e:
        rospy.logfatal(e)


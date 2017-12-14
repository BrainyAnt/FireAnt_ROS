#!/usr/bin/env python

import collections
#import time
#import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

#tree = ET.parse('config.xml')
#root = tree.getroot()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#define control pins
STAND_BY = 38
MOTORS = collections.namedtuple('motor', 'in1 in2 pwm_pin')
LEFT_MOTOR = MOTORS(16, 18, 12)
RIGHT_MOTOR = MOTORS(29, 31, 35)

class Motor:
    def __init__(self, pin1, pin2, pin3, frequency):
        self.in1 = pin1
        self.in2 = pin2
        self.pwm = pin3
        self.freq = frequency
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm, GPIO.OUT)
        self.duty = GPIO.PWM(self.pwm, self.freq)
        self.duty.start(0)
    
    def on(self):
        self.duty.start(0)
    
    def off(self):
        self.duty.stop()

    def set_forward_dir(self):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def set_backward_dir(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

    def run_motor(self, pwm):
        self.on()
        if pwm < 0:
            self.set_backward_dir()
        else:
            self.set_forward_dir()
        self.duty.ChangeDutyCycle(abs(pwm))

class DualMotorDriver:
    def __init__(self, pin, motor1, motor2):
        self.standby = pin
        self.left_motor = motor1
        self.right_motor = motor2

    def activate(self):
        self.left_motor.on()
        self.right_motor.on()
        GPIO.output(self.standby, GPIO.HIGH)

    def stand_by(self):
        GPIO.output(self.standby, GPIO.LOW)

    def run_robot(self, left, right, fwd, back):
        self.activate()
        #forward or backward?
        accelleration = (fwd - back) #between [-100,100]
        #left or right?
        drift = (right - left) #between [-100,100]
        #left motor power
        left_power = (accelleration + drift)*100/(100+abs(drift)) #between [-100,100]
        #right motor power
        right_power = (accelleration - drift)*100/(100+abs(drift)) #between [-100,100]
        self.left_motor.run_motor(left_power)
        self.right_motor.run_motor(right_power)

    def stop(self):
        self.stand_by()
        self.left_motor.off()
        self.right_motor.off()

motorL = Motor(LEFT_MOTOR.in1, LEFT_MOTOR.in2, LEFT_MOTOR.pwm_pin, 100)
motorR = Motor(RIGHT_MOTOR.in1, RIGHT_MOTOR.in2, RIGHT_MOTOR.pwm_pin, 100)
ROBOT = DualMotorDriver(STAND_BY, motorL, motorR)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)
    try:
        back = data.data['back']
    except ValueError:
        back = 0
        print("TYPE ERROR: back!")
    try:
        fwd = data.data['fwd']
    except ValueError:
        fwd = 0
        print("TYPE ERROR: fwd!")
    try:
        left = data.data['left']
    except ValueError:
        left = 0
        print("TYPE ERROR: left!")
    try:
        #right = int(data.data.split("u'")[3].split(":")[1].split(" ")[1].split(",")[0])
        right = data.data['right']
    except ValueError:
        right = 0
        print("TYPE ERROR: right!")
    ROBOT.run_robot(left, right, fwd, back)

def motion_topic_listener():
    rospy.init_node('motor_lobe', anonymous=True)
    rospy.Subscriber('motion', String, callback, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    ROBOT.activate()
    ROBOT.stand_by()
    
    motion_topic_listener()
    
    ROBOT.stop()
    GPIO.cleanup()

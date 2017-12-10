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
MOTORS = collections.namedtuple('motor', 'pwm_pin speed in1 in2')
LEFT_MOTOR = MOTORS(12, GPIO.PWM(12, 100), 16, 18)
RIGHT_MOTOR = MOTORS(35, GPIO.PWM(35, 100), 29, 31)

def motor_setup():
    GPIO.setup(STAND_BY, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR.pwm_pin, GPIO.OUT)
    LEFT_MOTOR.speed = GPIO.PWM(LEFT_MOTOR.pwm_pin,100)
    GPIO.setup(LEFT_MOTOR.in1, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR.in2, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR.pwm_pin, GPIO.OUT)
    RIGHT_MOTOR.speed = GPIO.PWM(RIGHT_MOTOR.pwm_pin,100)
    GPIO.setup(RIGHT_MOTOR.in1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR.in2, GPIO.OUT)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)
    try:
        #back = int(data.data.split("u'")[0].split(":")[1].split(" ")[1].split(",")[0])
        back = data.data['back']
    except ValueError:
        back = 0
        print("TYPE ERROR: back!")
    try:
        #fwd = int(data.data.split("u'")[1].split(":")[1].split(" ")[1].split(",")[0])
        fwd = data.data['fwd']
    except ValueError:
        fwd = 0
        print("TYPE ERROR: fwd!")
    try:
        #left = int(data.data.split("u'")[2].split(":")[1].split(" ")[1].split(",")[0])
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
    runRobot(left, right, fwd, back)

def motion_topic_listener():
    rospy.init_node('motor_lobe', anonymous=True)
    rospy.Subscriber('motion', String, callback, queue_size=5)
    rospy.spin()

def activate():
    GPIO.output(STAND_BY, GPIO.HIGH)

def standBy():
    GPIO.output(STAND_BY, GPIO.LOW)

def setForwardDir(motor):
    GPIO.output(motor.in1, GPIO.HIGH)
    GPIO.output(motor.in2, GPIO.LOW)

def setBackwardDir(motor):
    GPIO.output(motor.in1, GPIO.LOW)
    GPIO.output(motor.in2, GPIO.HIGH)

def runMotor(motor, pwm):
    if pwm < 0:
        setBackwardDir(motor)
    else:
        setForwardDir(motor)
    motor.speed.ChangeDutyCycle(abs(pwm))

def runRobot(left, right, fwd, back):
    activate()
    #forward or backward?
    accelleration = (fwd - back) #between [-100,100]
    #left or right?
    drift = (right - left) #between [-100,100]
    #left motor power
    left_power = (accelleration + drift)*100/(100+abs(drift)) #between [-100,100]
    #right motor power
    right_power = (accelleration - drift)*100/(100+abs(drift)) #between [-100,100]
    runMotor(LEFT_MOTOR, left_power)
    runMotor(RIGHT_MOTOR, right_power)

def stop():
    standBy()
    GPIO.output(LEFT_MOTOR.in1, GPIO.LOW)
    GPIO.output(LEFT_MOTOR.in2, GPIO.LOW)
    LEFT_MOTOR.speed.ChangeDutyCycle(0)
    GPIO.output(RIGHT_MOTOR.in1, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR.in2, GPIO.LOW)
    RIGHT_MOTOR.speed.ChangeDutyCycle(0)

def clear():
    GPIO.cleanup()

if __name__ == '__main__':
    motor_setup()
    motion_topic_listener()
    stop()
    clear()

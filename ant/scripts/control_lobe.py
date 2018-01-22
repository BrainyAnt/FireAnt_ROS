#!/usr/bin/env python

import sys
import ast
#import time
#import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

#tree = ET.parse('config.xml')
#root = tree.getroot()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

LED_PIN = 17
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

MOTOR_L_PIN_1 = 9
MOTOR_L_PIN_2 = 10
MOTOR_R_PIN_1 = 11
MOTOR_R_PIN_2 = 12
GPIO.setup(MOTOR_L_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_L_PIN_2, GPIO.OUT)
GPIO.setup(MOTOR_R_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_R_PIN_2, GPIO.OUT)
GPIO.output(MOTOR_L_PIN_1, GPIO.LOW)
GPIO.output(MOTOR_L_PIN_2, GPIO.LOW)
GPIO.output(MOTOR_R_PIN_1, GPIO.LOW)
GPIO.output(MOTOR_R_PIN_2, GPIO.LOW)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)
    control_data = ast.literal_eval(data.data)
    try:
        leds = control_data['leds']
        movement = {
            "fwd": control_data['movement']['fwd'],
            "back": control_data['movement']['back'],
            "left": control_data['movement']['left'],
            "right": control_data['movement']['right']
            }
    except ValueError:
        print("TYPE ERROR")
    switch_leds(leds)
    move(movement)

def switch_leds(led_states):
    if led_states['left'] is True:
        GPIO.output(LED_PIN, GPIO.HIGH)
    else:
        GPIO.output(LED_PIN, GPIO.LOW)

def move(movement):
    for i in movement:
        print("{}: {}".format(i, movement[i]))
    if int(movement["fwd"]) == 1:
        foreward()

def stop():
    GPIO.output(MOTOR_L_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_L_PIN_2, GPIO.LOW)
    GPIO.output(MOTOR_R_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_R_PIN_2, GPIO.LOW)

def foreward():
    GPIO.output(MOTOR_L_PIN_1, GPIO.HIGH)
    GPIO.output(MOTOR_L_PIN_2, GPIO.LOW)
    GPIO.output(MOTOR_R_PIN_1, GPIO.HIGH)
    GPIO.output(MOTOR_R_PIN_2, GPIO.LOW)

def back():
    GPIO.output(MOTOR_L_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_L_PIN_2, GPIO.HIGH)
    GPIO.output(MOTOR_R_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_R_PIN_2, GPIO.HIGH)

def left():
    GPIO.output(MOTOR_L_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_L_PIN_2, GPIO.HIGH)
    GPIO.output(MOTOR_R_PIN_1, GPIO.HIGH)
    GPIO.output(MOTOR_R_PIN_2, GPIO.LOW)

def right():
    GPIO.output(MOTOR_L_PIN_1, GPIO.HIGH)
    GPIO.output(MOTOR_L_PIN_2, GPIO.LOW)
    GPIO.output(MOTOR_R_PIN_1, GPIO.LOW)
    GPIO.output(MOTOR_R_PIN_2, GPIO.HIGH)

if __name__ == '__main__':
    try:
        rospy.init_node('control_lobe', anonymous=True)
        sub = rospy.Subscriber('control', String, callback, queue_size=10)
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exited with keyboard interrupt!")
        exit(0)
    GPIO.cleanup()

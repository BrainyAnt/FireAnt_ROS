#!/usr/bin/env python

import sys
import os
import ast
import collections
#import time
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

try:
    DIR = os.path.dirname(os.path.realpath(__file__))
    print(DIR)
    TREE = ET.parse(DIR+'/pin_config.xml')
except IOError:
    print("Config file not found!")
    sys.exit()
ROOT = TREE.getroot()

#get sensor pin config
PIN_CONFIG = ROOT[1]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#GPIO pin setup
SENSE_CONFIG = {}
for item in PIN_CONFIG:
    SENSE_CONFIG[item.tag] = int(item.text)
    GPIO.setup(SENSE_CONFIG[item.tag] , GPIO.IN)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)
    control_data = ast.literal_eval(data.data)
    try:
        sensors = control_data['sensors']
    except ValueError:
        print("TYPE ERROR")
    handle_sensors(sensors)

def sensor_reading_publish(data):
    rospy.loginfo(data)
    global SENSE_PUB
    SENSE_PUB.publish(str(data))

def handle_sensors(sensors):
    data = {}
    for sensor in sensors:
        if sensors[sensor]['request'] is True:
            reading = read_sensor(sensor)
            data[sensor] = {'value': reading}
            sensor_reading_publish(data)

def read_sensor(sensor_name):
    sensor_reading = GPIO.input(SENSE_CONFIG[sensor_name]) #use a function specific for each sensor
    return sensor_reading

if __name__ == '__main__':
    try:
        rospy.init_node('sensor_lobe', anonymous=True)
        sub = rospy.Subscriber('control', String, callback, queue_size=10)
        SENSE_PUB = rospy.Publisher('sense', String, queue_size=5)
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exited with keyboard interrupt!")
        exit(0)
    GPIO.cleanup()

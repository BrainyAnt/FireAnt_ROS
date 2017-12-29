#!/usr/bin/env python

import collections
#import time
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

TREE = ET.parse('pin_config.xml')
ROOT = TREE.getroot()

#get sensor pin config
PIN_CONFIG = ROOT["in"]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# pin config

#GPIO pin setup
for item in PIN_CONFIG:
    GPIO.setup(PIN_CONFIG[item] , GPIO.IN)

#Subscribe to sensor request topic
    #Send sensor data to topic on request
def sensor_request_listener():
    rospy.init_node('sensor_lobe', anonymous=True)
    rospy.Subscriber('sense_request', String, callback, queue_size=5)
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)
    #identify sensor
    handle_sensors(data.data)

SENSE_PUB = rospy.Publisher('sense_reading', String, queue_size=5)

def sense_reading_publish(data):
    rospy.loginfo(data)
    SENSE_PUB.publish(str(data))

def handle_sensors(message):
    data = None
    for sensor_name in message:
        if message[sensor_name] is True:
            reading = read_sensor(sensor_name)
            data.append({sensor_name: reading})
            sense_reading_publish(data)

def read_sensor(sensor_name):
    sensor_reading = GPIO.input(PIN_CONFIG[sensor_name])
    return sensor_reading

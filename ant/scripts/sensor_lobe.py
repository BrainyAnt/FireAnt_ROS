#!/usr/bin/env python

import sys
import ast
import collections
#import time
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

TREE = ET.parse('pin_config.xml')
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

#Subscribe to sensor request topic
    #Send sensor data to topic on request
def control_topic_listener():
    rospy.init_node('sensor_lobe', anonymous=True)
    rospy.Subscriber('control', String, callback, queue_size=10)
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + data.data)
    control_data = ast.literal_eval(data.data)
    try:
        sensors = control_data['sensors']
    except ValueError:
        print("TYPE ERROR")
    handle_sensors(sensors)

SENSE_PUB = rospy.Publisher('sense', String, queue_size=5)

def sensor_reading_publish(data):
    rospy.loginfo(data)
    SENSE_PUB.publish(str(data))

def handle_sensors(sensors):
    data = None
    for sensor in sensors:
        if sensors[sensor]['request'] is True:
            reading = read_sensor(sensor)
            data.append({sensor: {'value': reading}})
            sensor_reading_publish(data)

def read_sensor(sensor_name):
    sensor_reading = GPIO.input(SENSE_CONFIG[sensor_name]) #use a function specific for each sensor
    return sensor_reading

if __name__ == '__main__':
    try:    
        while True:
            control_topic_listener()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exited with keyboard interrupt!")
        exit(0)
    GPIO.cleanup()

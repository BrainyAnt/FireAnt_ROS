#!/usr/bin python

import sys
import time
import pyrebase
import rospy
from std_msgs.msg import String

CONFIG = {
    "apiKey": "apiKey",
    "authDomain": "brainyant-2e30d.firebaseapp.com",
    "databaseURL": "https://brainyant-2e30d.firebaseio.com/",
    "storageBucket": "gs://brainyant-2e30d.appspot.com/",
    "serviceAccount": "./brainyant-a3fa8afc4ec3.json"
}

FIREBASE = pyrebase.initialize_app(CONFIG)
AUTH = FIREBASE.auth()
DB = FIREBASE.database()
motion_pub = rospy.Publisher('motion', String, queue_size=5)

def motion_topic_streamer(category,item):
    rospy.init_node('firebase_lobe', anonymous=True)
    rate = rospy.Rate(100) #10Hz
    motion_stream = DB.child(category).child(item).child('commands').order_by_key().stream(stream_handler, None)
    rate.sleep()
    try:
        motion_stream.close()
    except AttributeError:
        print("GOT ERROR!")
        pass

def stream_handler(message):
    #print(message["data"])
    rospy.loginfo(message["data"])
    motion_pub.publish(str(message["data"]))

if __name__ == '__main__':
    
    '''
    print('HELLO! Welcome to the BrainyAnt firebase database.')
    print('ROBOTS:')
    all_robots = DB.child("robots").order_by_key().get()
    for robot in all_robots.each():
        print(robot.key())
    print('USERS:')
    all_users = DB.child("users").order_by_key().get()
    for user in all_users.each():
        print(user.key())
    '''
    try:
        while not rospy.is_shutdown():
            motion_topic_streamer("users","01bt2af2a8aTSUhOHSWTgHbTWhx1")
    except rospy.ROSInterruptException:
        print("ERROR: ROS Interrupted")
        sys.exit()
    except KeyboardInterrupt:
        print("ERROR: Keyboard Interrupt detected!")
        sys.exit()

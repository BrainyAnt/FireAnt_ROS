#!/usr/bin/env python

import xml.etree.ElementTree as ET
import pyrebase
import rospy
from std_msgs.msg import String

tree = ET.parse('config.xml')
root = tree.getroot()

#owner ID 
#robot ID
#accesskey
#get TOKEN from POST on SERVER

ROBOT_NAME = 'azorel'
CONFIG = {
    "apiKey": "AIzaSyDC23ZxJ7YjwVfM0BQ2o6zAtWinFrxCrcI",
    "authDomain": "brainyant-2e30d.firebaseapp.com",
    "databaseURL": "https://brainyant-2e30d.firebaseio.com/",
    "storageBucket": "gs://brainyant-2e30d.appspot.com/",
    "serviceAccount": "./brainyant-a3fa8afc4ec3.json"
}

FIREBASE = pyrebase.initialize_app(CONFIG)
AUTH = FIREBASE.auth()
EMAIL = 'example@example.com'
PASSWORD = 'unsecurepasswd'
USER = AUTH.sign_in_with_email_and_password(EMAIL, PASSWORD)
DB = FIREBASE.database()

OID = root[0][0].text
RID = root[0][1].text
UID = OID #USER['localId']

#token = get token from POST on SERVER
#user = AUTH.sign_in_with_custom_token(token)

MOTION_PUB = rospy.Publisher('motion', String, queue_size=5)

def motion_topic_streamer():
    """Listen for changes in firebase ControlData"""
    rospy.init_node('firebase_lobe', anonymous=True)
    rate = rospy.Rate(10) #10Hz
    motion_stream = DB.child("users").child(OID).child("robots").child(RID).child('users').child(UID).child("ControlData").order_by_key().stream(stream_handler, None)
    rate.sleep()
    motion_stream.close()

def get_control_data():
    """Return ControlData values from firebase"""
    control_data = DB.child("users").child(OID).child("robots").child(RID).child('users').child(UID).child("ControlData").order_by_key().get()
    return control_data

def stream_handler(message):
    """Stream handler. Publish data to topic."""
    rospy.loginfo(message["data"])
    MOTION_PUB.publish(str(message["data"]))

def robot_set_online():
    """Set field value of isOnline to True"""
    DB.child('users').child(OID).child('robots').child(RID).child('profile').child('isOnline').set(True)

def robot_set_offline():
    """Set field value of isOnline to False"""
    DB.child('users').child(OID).child('robots').child(RID).child('profile').child('isOnline').set(False)

def robot_is_online():
    """Return field value of isOnline"""
    return DB.child('users').child(OID).child('robots').child(RID).child('profile').child('isOnline').get().val()

def get_robot_name():
    """Return robot description field value"""
    name = DB.child('users').child(OID).child('robots').child(RID).child('profile').child('name').get().val()
    return name

def get_robot_description():
    """Return robot description field value"""
    description = DB.child('users').child(OID).child('robots').child(RID).child('profile').child('description').get().val()
    return description

#def share_robot_with_user(robot, user_id, user_name):
#   """Share specified robot with specified user"""
#    data = {'ControlData': {'fwd': 0, 'back': 0, 'left': 0, 'right': 0}, 'user': user_name}
#    DB.child('users').child(OID).child('robots').child(robot).child(user_id).set(data)

if __name__ == '__main__':

    '''
    print('HELLO! Welcome to the BrainyAnt firebase database.')
    print('ROBOTS:')
    all_robots = DB.child("robots").order_by_key().get()
    for robot in all_robots.each():
        print(robot.key())
    '''

    #set recurring algorithm. 3 second interval.
    robot_set_online()
    
    print("User", UID)
    print("Owner", OID)

    print('{}: {}'.format(get_robot_name(), get_robot_description()))

    print(get_control_data().key())
    for item in get_control_data().val():
        print("{}: {}".format(item, get_control_data().val()[item]))

    try:
        while not rospy.is_shutdown():
            motion_topic_streamer()
    except rospy.ROSInterruptException:
        print "ERROR: ROS Interrupted"
    except KeyboardInterrupt:
        print "ERROR: Keyboard Interrupt detected!"

    robot_set_offline()
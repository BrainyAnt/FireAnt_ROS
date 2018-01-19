#!/usr/bin/env python

import os
import sys
import ast
import sched
from multiprocessing import Process
import time
import xml.etree.ElementTree as ET
import json
import urllib2
import pyrebase
import rospy
from std_msgs.msg import String

# Exception class definition
class TokenRequestExrror(Exception):
    """Could not retreive token exception."""
    pass
class InvalidTokenException(Exception):
    """Received invalid authentication token exception."""
    pass
class EmptyQueueException(Exception):
    """The user queue is empty."""
    pass
class PermissionDeniedException(Exception):
    """Permission denied"""
    pass
class NoUIDException(Exception):
    """NoUIDException"""
    pass
class UserOfflineException(Exception):
    """User is offline exception"""
    pass

# Parse .xml file
try:
    DIR = os.path.dirname(os.path.realpath(__file__))
    print(DIR)
    TREE = ET.parse(DIR+'/../config/firebase_config.xml')
except IOError:
    print("Config file not found!")
    sys.exit()
ROOT = TREE.getroot()

OID = ROOT[0][0].text
RID = ROOT[0][1].text
ACCESSKEY = ROOT[0][2].text

FIREBASE_CONFIG = {
    "apiKey": "AIzaSyDC23ZxJ7YjwVfM0BQ2o6zAtWinFrxCrcI",
    "authDomain": "brainyant-2e30d.firebaseapp.com",
    "databaseURL": "https://brainyant-2e30d.firebaseio.com/",
    "storageBucket": "gs://brainyant-2e30d.appspot.com/"
}

FIREBASE = pyrebase.initialize_app(FIREBASE_CONFIG)
AUTH = FIREBASE.auth()
DB = FIREBASE.database()

GET_TOKEN_DATA = {
    'ownerID': OID,
    'robotID': RID,
    'accessKey': ACCESSKEY
}

try:
    REQUEST = urllib2.Request('https://robots.brainyant.com:8080/robotLogin')
    REQUEST.add_header('Content-Type', 'application/json')
    RESPONSE = urllib2.urlopen(REQUEST, json.dumps(GET_TOKEN_DATA))
    TOKEN = json.loads(RESPONSE.read())['customToken']
    if TOKEN is None:
        raise TokenRequestExrror
except TokenRequestExrror:
    print('Error! Could not retreive signin token from server. Server might be down.')

try:
    USERID = None
    USER = AUTH.sign_in_with_custom_token(TOKEN)
    REFRESH = AUTH.refresh(USER['refreshToken'])
    USERID = REFRESH['userId']
    IDTOKEN = REFRESH['idToken']
    if USERID is None:
        raise InvalidTokenException
except InvalidTokenException:
    print('Can not sign in to firebase. Invalid token.')

# ROS publisher
MOTION_PUB = None #rospy.Publisher('control', String, queue_size=10)
RATE = None

def init_talker():
    global MOTION_PUB
    MOTION_PUB = rospy.Publisher('control', String, queue_size=10)
    rospy.init_node('firebase_lobe', anonymous=True)
    global RATE
    RATE = rospy.Rate(10) # 10hz
    talk_thread = Process(target = motion_topic_streamer, args = [UID])
    return talk_thread

def motion_topic_streamer(userid):
    """Listen for changes in firebase ControlData"""
    motion_stream = DB.child('users').child(OID).child('robots').child(RID).child('users').child(userid).child("ControlData").order_by_key().stream(motion_stream_handler, IDTOKEN, None)
    global RATE
    RATE.sleep()
    motion_stream.close()

def motion_stream_handler(message):
    """Stream handler. Publish data to topic."""
    #rospy.loginfo(message["data"])
    print("STREAMING................................")
    global MOTION_PUB
    MOTION_PUB.publish(str(message["data"]))

# Sensor listener

def init_listener():
    global SUB
    SUB = rospy.Subscriber('sense', String, callback_sense, queue_size=10)
    listen_thread = Process(target = sensor_topic_listener)
    return listen_thread

def sensor_topic_listener():
    #rospy.spin()
    pass

def callback_sense(data):
    #rospy.loginfo(rospy.get_caller_id() + data.data)
    sense = ast.literal_eval(data.data)
    global UID
    update_sensor_value(UID, sense)

def update_sensor_value(userid, sense):
    #update firebase entry
    for sensor in sense:
        DB.child('users').child(OID).child('robots').child(RID).child('users').child(userid).child("ControlData").child("sensor").update({sensor: sense[sensor]}, token=IDTOKEN)


# DB user queue
def user_queue_streamer():
    """Listen for changes in firebase user queue"""
    queue_stream = DB.child('users').child(OID).child('robots').child(RID).child('queue').stream(queue_stream_handler, None)
    queue_stream.close()

def queue_stream_handler(message):
    """Queue stream handler. Get first user in queue"""
    print("[QUEUE_CHANGES]: {}".format(message["data"]))



def wait_for_users(u_entry, userid, uon):
    """Wait for user to show up in queue"""
    # Get UID
    print("Waiting for user ...")
    try:
        while userid is None:
            try:
                (userid, uon, u_entry) = get_first_user()
                if userid is None:
                    raise EmptyQueueException
            except EmptyQueueException:
                #print("[empty queue]")
                sys.stdout.write("[empty queue]\r")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("INTERRUPT!")
        exit(0)
    print("Found user --> UID: {}".format(userid))
    global UID
    UID = userid
    wait_for_user_on(u_entry, userid, uon)

def wait_for_user_on(u_entry, userid, uon):
    """Wait for current user to be online"""
    try:
        while not uon:
            try:
                uon = get_useron()
                if not uon:
                    raise UserOfflineException
            except UserOfflineException:
                #print('[user is offline]')
                sys.stdout.write("[user is offline]\r")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("INTERRUPT!")
        exit(0)
    print('User is online')

    print('Robot is on. Starting control session ...')
    set_robotOn(u_entry)
    set_startControl(u_entry)
    listen_for_commands(u_entry, userid, uon)

def listen_for_commands(u_entry, userid, user_on):
    """Listen for changes in ControlData"""
    #control = get_control_data(userid)
    try:
        #init_node
        rospy.init_node('firebase_lobe', anonymous=False)
        #publisher
        global MOTION_PUB
        MOTION_PUB = rospy.Publisher('control', String, queue_size=10)
        #subscriber
        global SUB
        SUB = rospy.Subscriber('sense', String, callback_sense, queue_size=10)
        #rate
        global RATE
        RATE = rospy.Rate(10)

        while user_on and not rospy.is_shutdown():
            motion_topic_streamer(userid)
            #talker = init_talker()
            #talker.start()
            #talker.join()
            #sensor_topic_listener()
            #listener = init_listener()
            #listener.start()
            #listener.join()
            user_on = get_useron()
    except rospy.ROSInterruptException:
        print("ERROR: ROS Interrupted")
        talker.join()
        listener.join()
    except KeyboardInterrupt:
        print("ERROR: Keyboard Interrupt detected!")
        talker.join()
        listener.join()
    
    print('Session ended.') #end of session
    print('Logging to archive') #log session
    log_session(u_entry, userid)


# DB still alive
def start_still_alive_every_n_secs(n):
    """Start a recurring function that signals the robot is online every n seconds"""
    s = sched.scheduler(time.time, time.sleep)
    s.enter(n, 1, still_alive, (s, n))
    s.run()

def still_alive(s, n):
    """Send a signal to the server every n seconds"""
    alive_data = {'robotID': RID}
    iamalive = urllib2.Request('https://robots.brainyant.com:8080/iAmAlive')
    iamalive.add_header('Content-Type', 'application/json')
    urllib2.urlopen(iamalive, json.dumps(alive_data))
    try:
        if is_online():
            s.enter(n, 1, still_alive, (s, n))
    except KeyboardInterrupt:
        print('Program terminated.')
        sys.exit()

# DB set and get functions
def get_control_data(userid):
    """Return ControlData values from firebase"""
    control_data = DB.child('users').child(OID).child('robots').child(RID).child('users').child(userid).child("ControlData").order_by_key().get(token=IDTOKEN)
    return control_data

def set_offline():
    """Set field value of isOnline to False"""
    DB.child('users').child(OID).child('robots').child(RID).child('profile').update({'isOnline': False}, token=IDTOKEN)

def is_online():
    """Return field value of isOnline"""
    return DB.child('users').child(OID).child('robots').child(RID).child('profile').child('isOnline').get(token=IDTOKEN).val()

def get_name():
    """Return robot name field value"""
    name = DB.child('users').child(OID).child('robots').child(RID).child('profile').child('name').get(token=IDTOKEN).val()
    return name

def get_description():
    """Return robot description field value"""
    description = DB.child('users').child(OID).child('robots').child(RID).child('profile').child('description').get(token=IDTOKEN).val()
    return description

def set_robotOn(entry):
    """Set robotOn flag to True"""
    DB.child('users').child(OID).child('robots').child(RID).child('queue').child(entry).update({"robotOn": True}, token=IDTOKEN)

def set_startControl(entry):
    """Record timestamp when control session starts"""
    timestamp = int(round(time.time()*1000)) #calendar.timegm(time.gmtime())
    DB.child('users').child(OID).child('robots').child(RID).child('queue').child(entry).update({"startControl": timestamp}, token=IDTOKEN)

def get_startControl(entry):
    """Return timestamp for start of control session"""
    return DB.child('users').child(OID).child('robots').child(RID).child('queue').child(entry).child('startControl').get(token=IDTOKEN).val()

def get_first_user():
    """Get first user information"""
    aux = DB.child('users').child(OID).child('robots').child(RID).child('queue').order_by_key().limit_to_first(1).get(token=IDTOKEN)
    try:
        for i in aux.each():
            uid = i.val()['uid']
            useron = i.val()['userOn']
            user_entry = i.key()
    except TypeError:
        #print("[empty queue"))
        return (None, None, None)
    return (uid, useron, user_entry)

def get_useron():
    """Get first user status"""
    aux = DB.child('users').child(OID).child('robots').child(RID).child('queue').order_by_key().limit_to_first(1).get(token=IDTOKEN)
    for i in aux.each():
        useron = i.val()['userOn']
    return useron

# DB log session
def log_session(u_entry, userID):
    """Log a session to archive after it is over"""
    log_timestamp = int(round(time.time()*1000)) #queue_archive entry: timestamp
    try:
        log_data = {
            log_timestamp: {
                'uid': userID, #uid
                'useTime': log_timestamp - get_startControl(u_entry), #useTime
                'waitTime': get_startControl(u_entry) - int(u_entry) #waitTime
            }
        }
    except ValueError:
        log_data = {
            log_timestamp: {
                'uid': UID, #uid
                'useTime': log_timestamp - get_startControl(USER_ENTRY), #useTime
                'waitTime': None #waitTime
            }
        }
    DB.child('users').child(OID).child('robots').child(RID).child('queueArchive').update(log_data, token=IDTOKEN)
    DB.child('users').child(OID).child('robots').child(RID).child('queue').child(USER_ENTRY).remove(token=IDTOKEN)

if __name__ == '__main__':
    p1 = Process(target = start_still_alive_every_n_secs, args = [1])
    p1.start()

    MOTION_PUB = None
    SUB = None
    RATE = None

    UID = None
    USERON = None
    USER_ENTRY = None
#wait for users
    try:
        while is_online():
            wait_for_users(USER_ENTRY, UID, USERON)
    except KeyboardInterrupt:
        print("Keyboard Interrupt!")
        p1.join()
        exit(0)
#cleanup
    p1.join()

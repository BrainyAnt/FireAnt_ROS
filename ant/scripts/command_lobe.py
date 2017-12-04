#!/usr/bin python

import sys, tty, termios
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

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(3)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == "__main__":
    fwd = 0
    back = 0
    right = 0
    left = 0
    print("GO:")
    input = raw_input(">>>")
    while not input == KeyboardInterrupt:
        print(input)
        input = raw_input(">>>")
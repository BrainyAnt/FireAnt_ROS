#!/usr/bin python

import sys, tty, termios
import time
import pyrebase
import rospy
from std_msgs.msg import String

CONFIG = {
    "apiKey": "AIzaSyDC23ZxJ7YjwVfM0BQ2o6zAtWinFrxCrcI",
    "authDomain": "brainyant-2e30d.firebaseapp.com",
    "databaseURL": "https://brainyant-2e30d.firebaseio.com/",
    "storageBucket": "gs://brainyant-2e30d.appspot.com/"
}

email = 'example@example.com'
password = 'unsecurepasswd'

FIREBASE = pyrebase.initialize_app(CONFIG)
AUTH = FIREBASE.auth()
DB = FIREBASE.database()
user = AUTH.sign_in_with_email_and_password(email, password)

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
    input = getch()
    while not input == KeyboardInterrupt:
        print(input)
        input = getch()
#!/usr/bin/env python

import qi
import sys
import argparse
import functools
import time
import rospy
from robobreizh_demo_components.srv import PepperSpeech, PepperSpeechResponse

class SpeechManager():
    def __init__(self, session) :
        print("Pepper speech ready")
        self.__alAnimatedSpeech = session.service("ALAnimatedSpeech")
        self.listener()
        print("Pepper speech ready")

    def handle_speech(self, req):
        self.__alAnimatedSpeech.say(req.Text)
        return PepperSpeechResponse(True)
    

    def listener(self):
        rospy.init_node('pepper_speech')
        s = rospy.Service('pepper_speech', PepperSpeech, self.handle_speech)
        print("Ready to add two ints.")
        rospy.spin()


if __name__ == "__main__":
    # TODO: Fix Bug parser with roslaunch
    #parser = argparse.ArgumentParser()
    #parser.add_argument("--ip", type=str, default="127.0.0.1",
    #                    help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    #parser.add_argument("--port", type=int, default=9559,
    #                    help="Naoqi port number")

    #args = parser.parse_args()
    ip = "127.0.0.1"
    port = 9559
    session = qi.Session()
    
    try:
        session.connect("tcp://" + ip + ":" + str(port))
        speech_manager = SpeechManager(session)
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
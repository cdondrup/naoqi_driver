#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import naoqi_interface_classes.connection as con

from naoqi_driver_service_classes.tts_services import TTSServices
from naoqi_driver_service_classes.animated_speech_services import AnimatedSpeechServices
from naoqi_driver_service_classes.motion_services import MotionServices
from naoqi_driver_service_classes.behaviour_manager_services import BehaviourManagerServices
from naoqi_driver_service_classes.tracker_services import TrackerServices
from naoqi_driver_service_classes.tablet_services import TabletServices


class ServicesNode(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        super_ns = rospy.get_param("~super_ns", "/naoqi_driver")
        TTSServices(super_ns)
        AnimatedSpeechServices(super_ns)
        MotionServices(super_ns)
        BehaviourManagerServices(super_ns)
        TrackerServices(super_ns)
        TabletServices(super_ns)
        rospy.loginfo("... done")

if __name__ == "__main__":
    rospy.init_node("naoqi_driver_service_node")
    broker = con.create_broker(rospy.get_param("~nao_ip", "pepper"), rospy.get_param("~nao_port", 9559))
    s = ServicesNode(rospy.get_name())
    rospy.spin()
    con.shutdown_broker(broker)

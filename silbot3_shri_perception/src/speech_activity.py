#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_base.perception_base import PerceptionBase

class SpeechActivity(PerceptionBase):
    def __init__(self):
        super(SpeechActivity, self).__init__("speech_activity")
        rospy.loginfo('\033[94m[%s]\033[0m initialze done...'%rospy.get_name())

    def handle_start_speech(self, msg):
        self.raise_event(self.conf_data.keys()[0], event='start_of_speech', data={})

    def handle_end_speech(self, msg):
        self.raise_event(self.conf_data.keys()[0], event='end_of_speech', data={})

    def handle_silency_detected(self, msg):
        self.raise_event(self.conf_data.keys()[0], event='silency_detected', data={})

if __name__ == '__main__':
    m = SpeechActivity()
    rospy.spin()

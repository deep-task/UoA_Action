#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import json
from std_msgs.msg import String

class SilbotInterfaceNode:
    def __init__(self):
        rospy.Subscriber('/silbotExecution', String, self.handle_silbot_execution)
        self.pub_silbot_complition = rospy.Publisher('/silbotComplition', String, queue_size=10)

        rospy.loginfo('%s initialized...'%rospy.get_name())

    def handle_silbot_execution(self, msg):
        rospy.loginfo('-- silbot_execution received --')

        rospy.sleep(5)

        rospy.loginfo('-- silbot_execution completed --')
        self.pub_silbot_complition.publish(json.dumps({}))


if __name__ == '__main__':
    rospy.init_node('silbot_interface_node', anonymous=False)
    m = SilbotInterfaceNode()
    rospy.spin()

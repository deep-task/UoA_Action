#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import json
from std_msgs.msg import String
from mind_msgs.msg import Reply

class ActionInterfaceNode:
    def __init__(self):
        self.silbot_task_complition = False

        rospy.Subscriber('/perceptionResult', String, self.handle_perception_result)
        rospy.Subscriber('/taskExecution', String, self.handle_task_execution)
        self.pub_task_completed = rospy.Publisher('/taskComplition', String, queue_size=10)

        self.silbot_task_requested = False
        rospy.Subscriber('/scene_queue_empty', String, self.handle_silbot_complition)
        self.pub_silbot_execution = rospy.Publisher('/reply', Reply, queue_size=10)

        rospy.loginfo('%s initialized...'%rospy.get_name())

    def handle_perception_result(self, msg):
        rospy.loginfo('-- perception_result received --')
        rospy.loginfo(msg)
        rospy.loginfo('-'*20)

    def handle_silbot_complition(self, msg):
        if self.silbot_task_requested:
            self.silbot_task_complition = True

    def handle_task_execution(self, msg):
        rospy.loginfo('-- task_execution received --')

        req_task = Reply()
        req_task.header.stamp = rospy.Time.now()
        req_task.reply = 'Hello! Nice to meet you. <br=0> Hello Again.'

        self.silbot_task_complition = False
        rospy.sleep(0.2)
        self.silbot_task_requested = True
        self.pub_silbot_execution.publish(req_task)
        while not rospy.is_shutdown() and not self.silbot_task_complition:
            rospy.sleep(0.1)

        rospy.loginfo('-- task_execution completed --')
        self.pub_task_completed.publish(json.dumps({}))
        self.silbot_task_requested = False


if __name__ == '__main__':
    rospy.init_node('action_interface_node', anonymous=False)
    m = ActionInterfaceNode()
    rospy.spin()

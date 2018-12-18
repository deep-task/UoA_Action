#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import json
import pdb
from std_msgs.msg import String
from mind_msgs.msg import Reply

class ActionInterfaceNode:
    def __init__(self):
        self.silbot_task_complition = False

        rospy.Subscriber('/perceptionResult', String, self.handle_perception_result)
        rospy.Subscriber('/taskRequest', String, self.handle_task_execution)
        self.pub_task_completed = rospy.Publisher('/taskCompletion', String, queue_size=10)

        self.pub_gaze_focusing = rospy.Publisher('gaze_focusing', String, queue_size=10)

        self.silbot_task_requested = False
        rospy.Subscriber('/scene_queue_empty', String, self.handle_silbot_complition)
        self.pub_silbot_execution = rospy.Publisher('/reply_deprecated', Reply, queue_size=10)

        rospy.loginfo('%s initialized...'%rospy.get_name())

    def handle_perception_result(self, msg):
        pass

    def handle_silbot_complition(self, msg):
        if self.silbot_task_requested:
            self.silbot_task_complition = True

    def create_complete_jsonstr(self, action_id, behavior):
        current_time = rospy.get_rostime()
        jsonSTTFrame = {
            "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "UOA",
            "target": ["UOS"],
            "content": ["robot_action"]
            },
        "robot_action": {
            "id": int(action_id),
            "behavior": behavior,
            "result": "completion"
            }
        }
        return jsonSTTFrame
        

    def handle_task_execution(self, msg):
        rospy.loginfo(msg)
        recv_data = json.loads(msg.data)

        # filter json msg
        if 'uoa' not in [x.lower() for x in recv_data['header']['target']]:
            return 
        if 'robot_action' not in recv_data:
            rospy.logerror('message is for uoa but there is no robot_action key')
            return

        rospy.loginfo('!!-- task_execution received --')

        action_data = recv_data['robot_action']
        action_id = action_data['id']
        
        req_task = Reply()
        req_task.header.stamp = rospy.Time.now()
        
        task = action_data['behavior']
        if ':' in task:
            task = task.split(':')[0]
        

        if task in ['focusing', 'gaze', 'listen_to_human']:
            
            rospy.loginfo(action_data['behavior'].encode('utf-8'))

            data = action_data['behavior'].split(':')
            encode_data = u' '.join(data).encode('utf-8')
            pub_target = String()
            if data[1] != 'end':
                pub_target.data = 'persons:' + action_data['user']
            else:
                pub_target.data = ''
            self.pub_gaze_focusing.publish(pub_target)

            rospy.sleep(0.5)

            jsonSTTFrame = self.create_complete_jsonstr(action_id, action_data['behavior'])
            self.pub_task_completed.publish(json.dumps(jsonSTTFrame))
            rospy.loginfo('-- task_execution completed --')
            return
        elif task in ['head_toss_gaze']:
            # gaze a person and back to neutral
            rospy.loginfo("received head_toss_gaze topic")

            data = action_data['behavior'].split(':')
            encode_data = u' '.join(data).encode('utf-8')
            pub_target = String()
            if data[1] != 'end':
                pub_target.data = 'persons:' + action_data['user']
            else:
                pub_target.data = ''
            self.pub_gaze_focusing.publish(pub_target)

            rospy.sleep(0.5)

            def stop_gazing(event):            
                pub_target.data = ''
                self.pub_gaze_focusing.publish(pub_target)
                rospy.loginfo('published topic to stop gazing(%s)' % action_id )

                jsonSTTFrame = self.create_complete_jsonstr(action_id, action_data['behavior'])
                rospy.loginfo(jsonSTTFrame)
                self.pub_task_completed.publish(json.dumps(jsonSTTFrame))

            rospy.Timer(rospy.Duration(3), stop_gazing, oneshot=True)
            return

        elif task in ['going_back_to_stand_by_place', 'approach_to_the_user']:
            data = action_data['behavior'].split(':')
            req_task.reply = '<mobility=%s>'%(data)
            self.pub_silbot_execution.publish(req_task)
            return
        elif task in ['saying_hello', 'initiation_of_conversation', 'continuation_of_conversation', 'termination_of_conversation', 'elicit_interest', 'saying_good_bye']:
            req_task.reply = '<gaze=persons:%s>'%action_data['user'] + '<sm=tag:%s>'%action_data['sm'] + action_data['dialog'] 
        elif task in ['action']:
            req_task.reply = '<sm=tag:%s>'%action_data['sm'] + action_data['dialog']
        else:
            rospy.info('task(%s) is not defined' % task)
            return

        self.silbot_task_complition = False
        rospy.sleep(0.1)
        self.silbot_task_requested = True
        self.pub_silbot_execution.publish(req_task)

        # block until requested action will be done
        while not rospy.is_shutdown() and not self.silbot_task_complition:
            rospy.sleep(0.1)

        jsonSTTFrame = self.create_complete_jsonstr(action_id, "action")

        rospy.loginfo('-- task_execution completed --')
        self.pub_task_completed.publish(json.dumps(jsonSTTFrame))
        self.silbot_task_requested = False


if __name__ == '__main__':
    rospy.init_node('action_interface_node', anonymous=False)
    m = ActionInterfaceNode()
    rospy.spin()

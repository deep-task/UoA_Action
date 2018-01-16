#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import json
from perception_base.perception_base import PerceptionBase

class PersonDetector(PerceptionBase):
    def __init__(self):
        super(PersonDetector, self).__init__("person_detector")
        rospy.loginfo('\033[94m[%s]\033[0m initialze done...'%rospy.get_name())

    def handle_person_detected(self, msg):
        recv_data = json.loads(msg.data)
        try:
            human_recog_data = recv_data['human_recognition']
        except KeyError as e:
            return

        write_data = self.conf_data['persons']['data']
        write_data['name'] = human_recog_data['name']
        write_data['description'] = human_recog_data['name']
        write_data['xyz'] = [human_recog_data['loc_x'], human_recog_data['loc_y'], human_recog_data['loc_z']]
        write_data['rpy'] = [0.0, 0.0, 0.0]
        write_data['frame_id'] = human_recog_data['frame_id']

        self.save_to_memory('persons', data=write_data)


if __name__ == '__main__':
    m = PersonDetector()
    rospy.spin()

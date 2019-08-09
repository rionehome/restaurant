# !/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from tfpose_ros.msg import *
from sensor_msgs.msg import Image


class RaiseHand:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        
        rospy.init_node("raise_hand_human")
        rospy.Subscriber("/pose_estimator/pose_3d", Persons, self.pose_3d_callback)
        rospy.Subscriber("/tf_pose/kinect_image", Image, self.kinect_image_callback)
        self.raise_hand_position_pub = rospy.Publisher('/restaurant/raise_hand_position', String, queue_size=10)
    
    def pose_3d_callback(self, msgs):
        # type: (Persons)->None
        for person in msgs.persons:
            self.is_raise_hand(person.body_part)
    
    def kinect_image_callback(self, msgs):
        # type: (Image)->None
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msgs, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return
    
    @staticmethod
    def is_raise_hand(body_part):
        # type: (dict)->None
        """
        腕を上げているかの判定
        右肩から 2,3,4
        左肩から 5,6,7
        :return:
        """
        __body_part_dict__ = {}
        for body in body_part:
            __body_part_dict__.setdefault(int(body.part_id), [body.x, body.y, body.z])
        
        if 3 in __body_part_dict__ and 4 in __body_part_dict__:
            if __body_part_dict__[3][1] > __body_part_dict__[4][1]:
                print "右腕が上がっている"
        
        if 6 in __body_part_dict__ and 7 in __body_part_dict__:
            if __body_part_dict__[6][1] > __body_part_dict__[7][1]:
                print "左腕が上がっている"


if __name__ == '__main__':
    raise_hand = RaiseHand()
    rospy.spin()

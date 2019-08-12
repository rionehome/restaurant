#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import rospy
from location.msg import Location
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose


class Data:
    def __init__(self, location, markers):
        # type: (Location, Marker) -> None
        self.location = location
        self.markers = markers


class RvizMarker:
    
    def __init__(self):
        # type: () -> None
        self.locations = {}
        self.marker_id = 100
        self.frame_id = "odom"
        self.ns = "location_markers"
        
        self.publisher = rospy.Publisher("location/marker", Marker, queue_size=10)
    
    def create_marker(self, message):
        # type: (Pose) -> []
        """
        Rvizで表示するマーカーの作成を行う
        :param message:
        :return:
        """
        r = random.random()
        g = random.random()
        b = random.random()
        scale = Vector3(0.5, 0.5, 0.5)
        
        marker_sphere = Marker()
        marker_sphere.header.stamp = rospy.Time.now()
        marker_sphere.header.frame_id = self.frame_id
        marker_sphere.ns = self.ns
        marker_sphere.id = self.marker_id
        marker_sphere.type = Marker.SPHERE
        marker_sphere.action = Marker.ADD
        marker_sphere.pose = message
        marker_sphere.scale = scale
        marker_sphere.color = ColorRGBA(r, g, b, 0.5)
        
        return [marker_sphere]
    
    def register(self, message):
        # type: (Pose) -> None
        """
        Rvizに表示するデータを保存し、Rvizにも認識出来る形でPublishする
        :param message: 場所データ
        :return: なし
        """
        markers = self.create_marker(message)
        for marker in markers:
            self.publisher.publish(marker)
    
    def delete(self, message):
        # type: (String) -> None
        """
        登録されたデータを消す
        ただし、保存されていない場合は警告だけして無視
        :param message: 場所名
        :return: なし
        """
        if message.data in self.locations:
            data = self.locations.get(message.data)
            if data is None:
                print("[UNREGISTERED] {}".format(data.location.name))
                return
            print("[Delete] {} ({}, {})".format(data.location.name, data.x, data.y))
            for marker in data.markers:
                marker.action = Marker.DELETE
                self.publisher.publish(marker)

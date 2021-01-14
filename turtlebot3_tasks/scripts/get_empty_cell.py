#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import UInt16
from tbot3_msgs.srv import (GetRandomCoordinate, GetRandomCoordinateResponse)

import math
import random

class GetIndex:
    def __init__(self):
        rospy.init_node("get_random_crd_server")
        rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
        self.get_random_crd = rospy.Service('get_random_crd', GetRandomCoordinate, self.handle_get_random_crd)
        self.indexes = []
        self.m_data = {
            "w" : 0,
            "h" : 0,
            "r" : 0.0
        }
        self.r_cs = {
            "x" : 0.0,
            "y" : 0.0
        }

    def handle_get_random_crd(self, req):
        self.convert_index_to_crd(self.indexes[random.randint(0, len(self.indexes)-1)])
        return GetRandomCoordinateResponse([self.r_cs["x"], self.r_cs["y"]])

    def convert_index_to_crd(self, index):
        # find row and column number
        r_n = math.floor(index / self.m_data["w"])
        c_n = index - r_n * self.m_data["w"]
        # find robot coordinates
        self.r_cs["x"] = (math.floor(self.m_data["h"] / 2.0) - r_n) * self.m_data["r"]
        self.r_cs["y"] = (math.floor(self.m_data["w"] / 2.0) - c_n) * self.m_data["r"]

    def callback_map(self, msg):
        self.m_data["w"] = msg.info.width
        self.m_data["h"] = msg.info.height
        self.m_data["r"] = msg.info.resolution
        # get empty cell indexes
        self.indexes = [i for i, j in enumerate(msg.data) if j == 0]

if __name__ == '__main__':
    random.seed(10)
    GetIndex()
    rospy.spin()

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import sys
import rospy
import yaml

from PIL import Image

from pyquaternion import Quaternion
#sudo pip install pyquaternion

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

current_pose = PoseStamped()

class StartUp(object):
    def __init__(self):
        global map_data
        self.sub = rospy.Subscriber("current_pose", PoseStamped, self.callback,queue_size=10)
        self.pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.start_flag = 0
        self.initial_map_data = map_data

    def callback(self, cp):
        global current_pose
        current_pose = cp
        if self.start_flag == 0:
            print("publish initial map")
            self.pub.publish(self.initial_map_data)
            self.start_flag = 1

class MapChanger(object):
    def __init__(self):
        global yaml_file_1F,yaml_file_2F,yaml_file_3F
        # self.sub = rospy.Subscriber("current_pose", PoseStamped, self.callback,queue_size=10)
        self.sub = rospy.Subscriber("floor", String, self.callback,queue_size=10)
        self.pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.pub2 = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.map1F = yaml_file_1F
        self.map2F = yaml_file_2F
        self.map3F = yaml_file_3F

    def callback(self,data):
        global map_data,current_pose
        print(data)
        map_data = OccupancyGrid()
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.get_rostime()
        initial_pose.pose.pose = current_pose.pose
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.06854
        if data.data == "1f":
            map_data = get_map_info(self.map1F)
            initial_pose.pose.pose.position.z = 0
        elif data.data == "2f":
            map_data = get_map_info(self.map2F)
            initial_pose.pose.pose.position.z = 0# input height of 2F
        elif data.data == "3f":
            map_data = get_map_info(self.map3F)
            initial_pose.pose.pose.position.z = 0# input height of 3F
        else:
            print("no map data")
        self.pub.publish(map_data)
        self.pub2.publish(initial_pose)


yaml_file_1F = "/home/mol/catkin_ws/src/mecanumrover_samples/map/willowgarage_small.yaml"
yaml_file_2F = "/home/mol/catkin_ws/src/mecanumrover_samples/map/NiC2F_for_Navigation.yaml"
yaml_file_3F = "/home/mol/catkin_ws/src/mecanumrover_samples/map/NiC3F_for_Navigation.yaml"

def get_map_info(yaml_file):

    with open(yaml_file, "rt") as fp:
        text = fp.read()
        map_info = yaml.safe_load(text)

    #read img
    im = np.array(Image.open("/home/mol/catkin_ws/src/mecanumrover_samples/map/"+map_info['image']), dtype='int8')

    print('##########image info############')
    print('name: %s' % map_info["image"])
    print('size: %s' % str(im.shape))
    print('origin[x,y,yaw]: %s' % str(map_info["origin"]))
    print('resolution: %s' % str(map_info["resolution"]))
    print('################################')

    # set msg_info
    _map = OccupancyGrid()
    _map.header.stamp = rospy.get_rostime()
    _map.header.frame_id = "/map"
    _map.info.resolution = map_info["resolution"]
    _map.info.width = im.shape[1]
    _map.info.height = im.shape[0]
    _map.info.origin.position.x = map_info["origin"][0]
    _map.info.origin.position.y = map_info["origin"][1]

    rad = map_info["origin"][2]
    q = Quaternion(axis=[0,0,1], angle=rad).elements
    _map.info.origin.orientation.x = q[1]
    _map.info.origin.orientation.y = q[2]
    _map.info.origin.orientation.z = q[3]
    _map.info.origin.orientation.w = q[0]

    #change occupancy data
    im_py = np.flip(im,0).reshape(-1,1)
    im_py = np.where(im_py != -1,100,0)# convert [0] to [100] and [-1] to [0] for occupancy gridmap system
    _map.data = im_py
    return _map


if __name__== '__main__':
    rospy.init_node('map_chenger',anonymous=True)
    map_data = get_map_info(yaml_file_1F)
    start_up = StartUp()
    map_changer = MapChanger()
    rospy.spin()

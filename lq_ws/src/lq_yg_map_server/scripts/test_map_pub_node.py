#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import time

def SetObstacle(local_map, xStartIndex, yStartIndex, xBound, yBound):
    for y in range(yStartIndex, yStartIndex + yBound):  # 行索引
        # 列索引 for x in range(200, local_map.info.width-boundary*2):
        for x in range(xStartIndex, xStartIndex + xBound):
            index = x + y*local_map.info.width
            local_map.data[index] = 100

def TFformer(child,parent,X,Y):
    br = tf.TransformBroadcaster()
    dx = 0
    dy = 0
    dz = 0.0

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    tfQuaternion = tf.transformations.quaternion_from_euler(
        roll, pitch,yaw, axes='sxyz')

    # ros_time = rospy.Time.now()

    # secs = ros_time.secs
    # nsecs = ros_time.nsecs
    # Create ROS time object
    # ros_time = rospy.Time(secs)
    ros_time = rospy.Time.now()
    time_ms = 0
    time_us = 0
    time_ns = 0
    # secs = time_ms / 1000 + time_us / 1000000 + time_ns / 1000000000
    # nsecs = (time_ms % 1000) * 1000000 + (time_us %
    #          1000000) * 1000 + time_ns % 1000000000
    secs = ros_time.secs
    nsecs = ros_time.nsecs
    rosTimeStamp = rospy.Time(int(secs), nsecs)

    br.sendTransform((dx, dy, dz),  # 平移变换
    (tfQuaternion[0], tfQuaternion[1],tfQuaternion[2], tfQuaternion[3]),  # 旋转变换
    rosTimeStamp, child,parent)

def DataUpdating(map_pub, local_map, map_height, map_width,pianyi):
    """
    数据更新函数
    """
    # 时间戳
    current_time = rospy.Time.now()
    ros_time = rospy.Time.now()

    # secs = time_ms / 1000 + time_us / 1000000 + time_ns / 1000000000
    # nsecs = (time_ms % 1000) * 1000000 + (time_us %
    #          1000000) * 1000 + time_ns % 1000000000
    secs = ros_time.secs
    nsecs = ros_time.nsecs
    rosTimeStamp = rospy.Time(int(secs), nsecs)
    local_map.header.stamp = rosTimeStamp
    local_map.info.map_load_time = current_time
    local_map.header.frame_id = "marine_rader_map"
    local_map.info.resolution = 1  # 分辨率 1m  图片分片率(单位: m/像素)。
    local_map.info.height = map_height  # 长和宽 mm
    local_map.info.width = map_width
    local_map.info.origin.position.x = -map_width/2# 原点位置
    local_map.info.origin.position.y = -map_height/2
    # local_map.info.origin.position.x = 0# 原点位置
    # local_map.info.origin.position.y = 0

    local_map.info.origin.position.z = 0

    # rospy.logfatal("map_publish_node x = : %f", local_map.info.origin.position.x)  # [FATAL]
    # rospy.logfatal("map_publish_node y = : %f", local_map.info.origin.position.y)  # [FATAL]

    # local_map.info.origin.orientation.x =  0.985 # 原点姿态
    # local_map.info.origin.orientation.y = 0.174
    # local_map.info.origin.orientation.z =  0
    # local_map.info.origin.orientation.w =  0
    tfQuaternion = tf.transformations.quaternion_from_euler(
    0, 0, 0, axes='sxyz')
    local_map.info.origin.orientation.x =  tfQuaternion[0] # 原点姿态
    local_map.info.origin.orientation.y =  tfQuaternion[1]
    local_map.info.origin.orientation.z =  tfQuaternion[2]
    local_map.info.origin.orientation.w =  tfQuaternion[3]
    # 默认全部未知区域  # 全部为-1 -> 0
    # local_map.data = [-1] * local_map.info.width * local_map.info.height
    shape = local_map.info.width * local_map.info.height
    # print("local_map.data.shape = ", shape)
    local_map.data = np.full(shape, 0, dtype=int, order='C')
    # print(type(local_map.data))

    # # 设置障碍区域 x
    xBound = 5
    yBound = 5
    xStartIndex = 100
    yStartIndex = 100
    SetObstacle(local_map, xStartIndex, yStartIndex, xBound, yBound)  # 1#
    xStartIndex = 500
    yStartIndex = 500
    SetObstacle(local_map, xStartIndex, yStartIndex, xBound, yBound)  # 2#
    xStartIndex = 1000
    yStartIndex = 1000
    SetObstacle(local_map, xStartIndex, yStartIndex, xBound, yBound)  # 3#
    
    xStartIndex = 100
    yStartIndex = 500
    SetObstacle(local_map, xStartIndex, yStartIndex, xBound, yBound)  # 4#

    xStartIndex = 100
    yStartIndex = 1000
    SetObstacle(local_map, xStartIndex, yStartIndex, xBound, yBound)  # 5#

    SetObstacle(local_map, 500, 100, xBound, yBound)  # 6#

    SetObstacle(local_map, 500, 1000, xBound, yBound)  # 7#
    SetObstacle(local_map, 1000, 100, xBound, yBound)  # 8#
    SetObstacle(local_map, 1000, 500, xBound, yBound)  # 9#

    SetObstacle(local_map, 500, 1000, xBound, yBound)  # 7#
    SetObstacle(local_map, 1000, 100, xBound, yBound)  # 8#
    SetObstacle(local_map, 1000, 500, xBound, yBound)  # 9#

    for x in range(pianyi -1500, pianyi, 80):  # 行索引
        for y in range(500, 1500, 80):
            SetObstacle(local_map, x, y, 5, 5)  # 9#

    # 发布地图
    TFformer("/marine_map","map",0,0)
    TFformer("/marine_rader_map","map",0, 0)
    map_pub.publish(local_map)

def node():
    """
    节点启动函数
    """
    try:
        # 初始化节点MapPublish
        rospy.init_node('test_map_publish_node')
        # 定义发布器 map_pub 发布 local_map
        map_pub = rospy.Publisher(
            'marine_rader_map', OccupancyGrid, queue_size=1)

        # 定义地图
        local_map = OccupancyGrid()
        map_height = 400 * 5
        map_width = 400 * 5

        rospy.loginfo(
            "The program of ow_local_map_publish_node is running ...")
        # 初始化循环频率
        rate = rospy.Rate(1)
        # 在程序没退出的情况下
        pianyi = 0
        while not rospy.is_shutdown():
            # 数据更新函数
            DataUpdating(map_pub, local_map, map_height, map_width,pianyi)
            pianyi += 1
            if(pianyi == 120):
                pianyi = 0

            rate.sleep()
    except Exception as e:
        rospy.logfatal("map_publish_node has a Exception : %s", e)  # [FATAL]


if __name__ == '__main__':
    node()

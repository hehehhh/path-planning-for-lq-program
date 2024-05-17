#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
resulution = 5

def TFformer(child,parent,X,Y):
    br = tf.TransformBroadcaster()
    dx = X*resulution
    dy = Y*resulution

    dz = 0.0

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    tfQuaternion = tf.transformations.quaternion_from_euler(
        roll, pitch,yaw, axes='sxyz')
    ros_time = rospy.Time.now()

    # time_ms = 110
    # time_us = 12
    # time_ns = 13
    # secs = time_ms / 1000 + time_us / 1000000 + time_ns / 1000000000
    # nsecs = (time_ms % 1000) * 1000000 + (time_us %
    #          1000000) * 1000 + time_ns % 1000000000

    # Create ROS time object
    # ros_time = rospy.Time(secs)
    secs = ros_time.secs
    nsecs = ros_time.nsecs
    rosTimeStamp = rospy.Time(int(secs), nsecs)
    br.sendTransform((dx, dy, dz),  # 平移变换
    (tfQuaternion[0], tfQuaternion[1],tfQuaternion[2], tfQuaternion[3]),  # 旋转变换
    rosTimeStamp, child,parent)

def DataUpdating(obs_pub, obs_array,rate,frame,flag):
    maker = Marker()
    maker.header.frame_id = frame
    ros_time = rospy.Time.now()

    # time_ms = 110
    # time_us = 12
    # time_ns = 13
    # secs = time_ms / 1000 + time_us / 1000000 + time_ns / 1000000000
    # nsecs = (time_ms % 1000) * 1000000 + (time_us %
    #          1000000) * 1000 + time_ns % 1000000000
    secs = ros_time.secs
    nsecs = ros_time.nsecs
    rosTimeStamp = rospy.Time(int(secs), nsecs)
    maker.header.stamp = rosTimeStamp
    maker.type = Marker.SPHERE_LIST
    maker.action = Marker.ADD
    maker.scale.x = 200
    maker.scale.y = 200
    maker.scale.z = 200
    maker.color.a = 1.0
    maker.color.r = 1.0
    maker.color.g = 0
    maker.color.b = 0


    # maker.pose.position.x = (-400)*resulution
    # maker.pose.position.y = (-400)*resulution
    maker.pose.position.x = 0
    maker.pose.position.y = 0

    maker.pose.position.z = 0
    tfQuaternion = tf.transformations.quaternion_from_euler(
    0, 0, 0, axes='sxyz')  
    maker.pose.orientation.x = tfQuaternion[0]
    maker.pose.orientation.y = tfQuaternion[1]  
    maker.pose.orientation.z = tfQuaternion[2]
    maker.pose.orientation.w = tfQuaternion[3]

    if flag == 1:
        flag1 = 1
        flag2 = 0
    else:
        flag1 = 0
        flag2 = 1
    pt = Point()
    maker.points.clear()
    for i in range(5):

        x = -400 + (0 + i * 20 + rate)%800
        y = -400 + (800 - i * 20 - rate)%800

        pt.x = 0
        pt.y = 0
        pt.z = 0
        maker.pose.position.x = x*flag1*resulution
        maker.pose.position.y = y*flag2*resulution
        maker.points.append(pt)
    obs_array.markers.append(maker)
    TFformer(frame,"/map",0,0)
    obs_pub.publish(obs_array)
    obs_array.markers.clear()

def node():
    """
    节点启动函数
    """
    try:
        # 初始化节点MapPublish
        rospy.init_node('test_obstacle_pub_node')
        # 定义发布器 map_pub 发布 local_map
        radar_obs_pub = rospy.Publisher(
            'marine_radar_static_obs', MarkerArray, queue_size=1)
        camera_obs_pub = rospy.Publisher(
            'camera_static_obs', MarkerArray, queue_size=1)

        # 定义地图
        radar_obs_array = MarkerArray()
        camera_obs_array = MarkerArray()
        rate1 = 1
        rate2 = 3  
        frameid1 = "/marine_rader_map"
        frameid2 = "/marine_rader_map"
        rospy.loginfo(
            "The program of obstable array is running ...")
        # 初始化循环频率
        rate = rospy.Rate(1)
        # 在程序没退出的情况下
        while not rospy.is_shutdown():
            rate1 += rate1
            rate2 += rate2
            # 数据更新函数
            DataUpdating(radar_obs_pub, radar_obs_array,rate1,frameid1,1)
            DataUpdating(camera_obs_pub, camera_obs_array,rate2,frameid2,0)

            rate.sleep()
    except Exception as e:
        rospy.logfatal("map_publish_node has a Exception : %s", e)  # [FATAL]

if __name__ == '__main__':
    node()

#!/usr/bin/env python

import rospy
from platforms_server.msg import Aruco as aruco_msg, Point2d, MarkerCorners
from vision.MarkersAnalizer import MarkersAnalizer


def recognize_fields_object_by_id(data):
    markers = data.markers
    corners_list = list(marker.corners for marker in markers)
    ids = data.ids

    markers_dict = dict(zip(ids, corners_list))

    analizer.parse_fields_objects_by_id(markers_dict)
    robots = analizer.get_robots()
    goals = analizer.get_goals()
    obstacles = analizer.get_obstacles()

    return robots, goals, obstacles


def callback(data):
    robots, goals, obstacles = recognize_fields_object_by_id(data)

    print(robots)
    print(goals)
    print(obstacles)

analizer = MarkersAnalizer()
rospy.init_node('markers_analizer_node', anonymous=True)
markers_data_sub = rospy.Subscriber("detected_markers", aruco_msg, callback)
rospy.spin()

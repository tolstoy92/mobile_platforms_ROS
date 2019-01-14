#!/usr/bin/env python

import rospy
from platforms_server.msg import FieldObjects as FieldObjects_msg, Path, All_pathes
from vision.Fileds_objects import Robot, Obstacle, Goal
from path_planner.Planner import Paths_planner
from time import time


def callback(data):
    robots = data.robots
    obstacles = data.obstacles
    goals = data.goals
    planner.set_robots(robots)
    planner.set_targets(goals)
    planner.set_obstacles(obstacles)
    paths = planner.target_assignment()

    if len(paths):
        msg = prepare_msg(paths)
        # for p in msg.paths_list:
        #     for pt in p.path_points:
        #         print(pt.x)
        paths_data_publisher.publish(msg)
        # for path_id in paths.keys():
        #     actual_path = planner.path_to_point_list(paths[path_id])
        #     for pt in actual_path:
        #         print(pt.get_xy())


def prepare_msg(paths):
    all_pathes_msg = All_pathes()
    paths_list = []
    for path_id in paths.keys():
        path_msg = Path()
        actual_path = planner.path_to_point_list(paths[path_id])
        path_msg.path_points = list(pt for pt in actual_path)
        paths_list.append(path_msg)
    all_pathes_msg.paths_list = paths_list
    return all_pathes_msg

rospy.init_node("aruco_detector_node")
planner = Paths_planner()
# markers_data_publisher = rospy.Publisher("detected_markers", ArucoData)
objects_sub = rospy.Subscriber("field_objects", FieldObjects_msg, callback)
paths_data_publisher = rospy.Publisher("paths_data", All_pathes)

rospy.spin()
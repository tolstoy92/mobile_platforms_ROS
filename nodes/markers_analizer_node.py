#!/usr/bin/env python

import rospy
from platforms_server.msg import ArucoData, FieldObjects, AllPathes
from vision.MarkersAnalizer import MarkersAnalizer


analizer = MarkersAnalizer()

rospy.spin()

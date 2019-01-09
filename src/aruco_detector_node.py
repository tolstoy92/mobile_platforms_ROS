#!/usr/bin/env python


import rospy
import cv2
from cv2 import aruco
from platforms_server.msg import Aruco as aruco_msg, Point2d, MarkerCorners
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def corners_to_msg(corners):
    corners_msgs_list = []
    for corner in corners:
        corners_msg = MarkerCorners(tuple(Point2d(x, y) for x, y in corner[0]))
        corners_msgs_list.append(corners_msg)
    return corners_msgs_list


def callback(data):
    if not rospy.is_shutdown():
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            all_corners, ids_np, _ = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
            if not isinstance(ids_np, type(None)):
                markers_msg = aruco_msg()

                corners_msgs_list = corners_to_msg(all_corners)
                ids_msgs_list = list(id[0] for id in ids_np.tolist())

                markers_msg.markers = corners_msgs_list
                markers_msg.ids = ids_msgs_list

                markers_data_publisher.publish(markers_msg)

                aruco.drawDetectedMarkers(cv_image, all_corners, ids_np)

            cv2.imshow("cv_img", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)
    else:
        cv2.destroyAllWindows()


bridge = CvBridge()

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()

rospy.init_node('aruco_detector_node', anonymous=True)

markers_data_publisher = rospy.Publisher("detected_markers", aruco_msg)
image_sub = rospy.Subscriber("square_image", Image, callback)

rospy.spin()


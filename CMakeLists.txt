cmake_minimum_required(VERSION 2.8.3)
project(platforms_server)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)


add_message_files(
   FILES
   Aruco.msg
   MarkerCorners.msg
   Point2d.msg   
)


generate_messages(
   DEPENDENCIES
   std_msgs 
)


catkin_package(
   CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


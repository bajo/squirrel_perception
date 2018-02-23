cmake_minimum_required(VERSION 2.8.3)
project(squirrel_demo_behaviors)

find_package(catkin REQUIRED
    COMPONENTS geometry_msgs rospy smach tf uashh_smach
)

catkin_package(

    CATKIN_DEPENDS geometry_msgs rospy smach tf uashh_smach
)

catkin_install_python(
    PROGRAMS src/ikea.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
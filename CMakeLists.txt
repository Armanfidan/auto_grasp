cmake_minimum_required(VERSION 3.0.2)
project(auto_grasp)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    vision_msgs
    dynamic_reconfigure
)

# catkin_install_python(
    # PROGRAMS scripts/spherical_grasps_server.py
    # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

generate_dynamic_reconfigure_options(
    cfg/SphericalGrasp.cfg
)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES auto_grasp
    # CATKIN_DEPENDS roscpp rospy vision_msgs dynamic_reconfigure
    # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_custom_target(spherical_grasps_server ALL
    DEPENDS ${PROJECT_NAME}_gencfg 
)
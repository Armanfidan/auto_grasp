cmake_minimum_required(VERSION 3.0.2)
project(auto_grasp)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    vision_msgs
    dynamic_reconfigure
    tf
	roslaunch
)

# catkin_install_python(
    # PROGRAMS scripts/spherical_grasps_server.py
    # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_message_files(
   FILES
   Entity_msg.msg
   CurrentState.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AddFloorPlane.srv
   RemoveFloorPlane.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

generate_dynamic_reconfigure_options(
    cfg/SphericalGrasp.cfg
)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES auto_grasp
    # CATKIN_DEPENDS roscpp rospy vision_msgs dynamic_reconfigure tf
    # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_custom_target(spherical_grasps_server ALL
    DEPENDS ${PROJECT_NAME}_gencfg 
)

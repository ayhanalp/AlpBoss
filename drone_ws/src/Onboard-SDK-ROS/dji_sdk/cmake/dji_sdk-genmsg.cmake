# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dji_sdk: 8 messages, 27 services")

set(MSG_I_FLAGS "-Idji_sdk:/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dji_sdk_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv" "dji_sdk/MissionHotpointTask"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv" "dji_sdk/MissionHotpointTask"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg" "dji_sdk/MissionWaypoint:dji_sdk/MissionWaypointAction"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg" "dji_sdk/MissionWaypointAction"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg" "dji_sdk/Waypoint"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv" "dji_sdk/MissionWaypointTask:dji_sdk/MissionWaypoint:dji_sdk/MissionWaypointAction"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg" ""
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv" "dji_sdk/MissionWaypointTask:dji_sdk/MissionWaypoint:dji_sdk/MissionWaypointAction"
)

get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv" NAME_WE)
add_custom_target(_dji_sdk_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk" "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_msg_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)

### Generating Services
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)
_generate_srv_cpp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
)

### Generating Module File
_generate_module_cpp(dji_sdk
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dji_sdk_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dji_sdk_generate_messages dji_sdk_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_cpp _dji_sdk_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_sdk_gencpp)
add_dependencies(dji_sdk_gencpp dji_sdk_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_sdk_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_msg_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)

### Generating Services
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)
_generate_srv_lisp(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
)

### Generating Module File
_generate_module_lisp(dji_sdk
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dji_sdk_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dji_sdk_generate_messages dji_sdk_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_lisp _dji_sdk_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_sdk_genlisp)
add_dependencies(dji_sdk_genlisp dji_sdk_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_sdk_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_msg_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)

### Generating Services
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg;/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)
_generate_srv_py(dji_sdk
  "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
)

### Generating Module File
_generate_module_py(dji_sdk
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dji_sdk_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dji_sdk_generate_messages dji_sdk_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Activation.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_sdk_generate_messages_py _dji_sdk_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_sdk_genpy)
add_dependencies(dji_sdk_genpy dji_sdk_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_sdk_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dji_sdk_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(dji_sdk_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dji_sdk_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(dji_sdk_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dji_sdk_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(dji_sdk_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dji_sdk_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(dji_sdk_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk/.+/__init__.pyc?$"
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dji_sdk_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(dji_sdk_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dji_sdk_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(dji_sdk_generate_messages_py sensor_msgs_generate_messages_py)
endif()

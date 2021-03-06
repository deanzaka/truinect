# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pcl_tracking: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipcl_tracking:/home/deanzaka/Github/truinect/src/pcl_tracking/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pcl_tracking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg" NAME_WE)
add_custom_target(_pcl_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pcl_tracking" "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pcl_tracking
  "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_tracking
)

### Generating Services

### Generating Module File
_generate_module_cpp(pcl_tracking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_tracking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pcl_tracking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pcl_tracking_generate_messages pcl_tracking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg" NAME_WE)
add_dependencies(pcl_tracking_generate_messages_cpp _pcl_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_tracking_gencpp)
add_dependencies(pcl_tracking_gencpp pcl_tracking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_tracking_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pcl_tracking
  "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_tracking
)

### Generating Services

### Generating Module File
_generate_module_lisp(pcl_tracking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_tracking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pcl_tracking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pcl_tracking_generate_messages pcl_tracking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg" NAME_WE)
add_dependencies(pcl_tracking_generate_messages_lisp _pcl_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_tracking_genlisp)
add_dependencies(pcl_tracking_genlisp pcl_tracking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_tracking_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pcl_tracking
  "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_tracking
)

### Generating Services

### Generating Module File
_generate_module_py(pcl_tracking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_tracking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pcl_tracking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pcl_tracking_generate_messages pcl_tracking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/deanzaka/Github/truinect/src/pcl_tracking/msg/Pose.msg" NAME_WE)
add_dependencies(pcl_tracking_generate_messages_py _pcl_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_tracking_genpy)
add_dependencies(pcl_tracking_genpy pcl_tracking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_tracking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_tracking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(pcl_tracking_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(pcl_tracking_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_tracking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(pcl_tracking_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(pcl_tracking_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_tracking)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_tracking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_tracking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(pcl_tracking_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(pcl_tracking_generate_messages_py std_msgs_generate_messages_py)

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "all_om: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iall_om:/home/son/soon/maker/src/all_om/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(all_om_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/point.msg" NAME_WE)
add_custom_target(_all_om_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "all_om" "/home/son/soon/maker/src/all_om/msg/point.msg" ""
)

get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/points.msg" NAME_WE)
add_custom_target(_all_om_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "all_om" "/home/son/soon/maker/src/all_om/msg/points.msg" "all_om/point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(all_om
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_om
)
_generate_msg_cpp(all_om
  "/home/son/soon/maker/src/all_om/msg/points.msg"
  "${MSG_I_FLAGS}"
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_om
)

### Generating Services

### Generating Module File
_generate_module_cpp(all_om
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_om
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(all_om_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(all_om_generate_messages all_om_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/point.msg" NAME_WE)
add_dependencies(all_om_generate_messages_cpp _all_om_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/points.msg" NAME_WE)
add_dependencies(all_om_generate_messages_cpp _all_om_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_om_gencpp)
add_dependencies(all_om_gencpp all_om_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_om_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(all_om
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_om
)
_generate_msg_eus(all_om
  "/home/son/soon/maker/src/all_om/msg/points.msg"
  "${MSG_I_FLAGS}"
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_om
)

### Generating Services

### Generating Module File
_generate_module_eus(all_om
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_om
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(all_om_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(all_om_generate_messages all_om_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/point.msg" NAME_WE)
add_dependencies(all_om_generate_messages_eus _all_om_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/points.msg" NAME_WE)
add_dependencies(all_om_generate_messages_eus _all_om_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_om_geneus)
add_dependencies(all_om_geneus all_om_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_om_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(all_om
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_om
)
_generate_msg_lisp(all_om
  "/home/son/soon/maker/src/all_om/msg/points.msg"
  "${MSG_I_FLAGS}"
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_om
)

### Generating Services

### Generating Module File
_generate_module_lisp(all_om
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_om
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(all_om_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(all_om_generate_messages all_om_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/point.msg" NAME_WE)
add_dependencies(all_om_generate_messages_lisp _all_om_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/points.msg" NAME_WE)
add_dependencies(all_om_generate_messages_lisp _all_om_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_om_genlisp)
add_dependencies(all_om_genlisp all_om_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_om_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(all_om
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_om
)
_generate_msg_nodejs(all_om
  "/home/son/soon/maker/src/all_om/msg/points.msg"
  "${MSG_I_FLAGS}"
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_om
)

### Generating Services

### Generating Module File
_generate_module_nodejs(all_om
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_om
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(all_om_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(all_om_generate_messages all_om_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/point.msg" NAME_WE)
add_dependencies(all_om_generate_messages_nodejs _all_om_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/points.msg" NAME_WE)
add_dependencies(all_om_generate_messages_nodejs _all_om_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_om_gennodejs)
add_dependencies(all_om_gennodejs all_om_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_om_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(all_om
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_om
)
_generate_msg_py(all_om
  "/home/son/soon/maker/src/all_om/msg/points.msg"
  "${MSG_I_FLAGS}"
  "/home/son/soon/maker/src/all_om/msg/point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_om
)

### Generating Services

### Generating Module File
_generate_module_py(all_om
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_om
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(all_om_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(all_om_generate_messages all_om_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/point.msg" NAME_WE)
add_dependencies(all_om_generate_messages_py _all_om_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/son/soon/maker/src/all_om/msg/points.msg" NAME_WE)
add_dependencies(all_om_generate_messages_py _all_om_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_om_genpy)
add_dependencies(all_om_genpy all_om_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_om_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_om)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_om
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(all_om_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_om)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_om
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(all_om_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_om)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_om
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(all_om_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_om)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_om
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(all_om_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_om)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_om\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_om
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(all_om_generate_messages_py sensor_msgs_generate_messages_py)
endif()

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "x_msgs: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ix_msgs:/home/tom/x_os/src/x_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(x_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tom/x_os/src/x_msgs/srv/X_Command.srv" NAME_WE)
add_custom_target(_x_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "x_msgs" "/home/tom/x_os/src/x_msgs/srv/X_Command.srv" ""
)

get_filename_component(_filename "/home/tom/x_os/src/x_msgs/msg/X_Status.msg" NAME_WE)
add_custom_target(_x_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "x_msgs" "/home/tom/x_os/src/x_msgs/msg/X_Status.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(x_msgs
  "/home/tom/x_os/src/x_msgs/msg/X_Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/x_msgs
)

### Generating Services
_generate_srv_cpp(x_msgs
  "/home/tom/x_os/src/x_msgs/srv/X_Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/x_msgs
)

### Generating Module File
_generate_module_cpp(x_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/x_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(x_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(x_msgs_generate_messages x_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/x_os/src/x_msgs/srv/X_Command.srv" NAME_WE)
add_dependencies(x_msgs_generate_messages_cpp _x_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/x_os/src/x_msgs/msg/X_Status.msg" NAME_WE)
add_dependencies(x_msgs_generate_messages_cpp _x_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(x_msgs_gencpp)
add_dependencies(x_msgs_gencpp x_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS x_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(x_msgs
  "/home/tom/x_os/src/x_msgs/msg/X_Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/x_msgs
)

### Generating Services
_generate_srv_lisp(x_msgs
  "/home/tom/x_os/src/x_msgs/srv/X_Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/x_msgs
)

### Generating Module File
_generate_module_lisp(x_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/x_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(x_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(x_msgs_generate_messages x_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/x_os/src/x_msgs/srv/X_Command.srv" NAME_WE)
add_dependencies(x_msgs_generate_messages_lisp _x_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/x_os/src/x_msgs/msg/X_Status.msg" NAME_WE)
add_dependencies(x_msgs_generate_messages_lisp _x_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(x_msgs_genlisp)
add_dependencies(x_msgs_genlisp x_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS x_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(x_msgs
  "/home/tom/x_os/src/x_msgs/msg/X_Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/x_msgs
)

### Generating Services
_generate_srv_py(x_msgs
  "/home/tom/x_os/src/x_msgs/srv/X_Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/x_msgs
)

### Generating Module File
_generate_module_py(x_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/x_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(x_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(x_msgs_generate_messages x_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/x_os/src/x_msgs/srv/X_Command.srv" NAME_WE)
add_dependencies(x_msgs_generate_messages_py _x_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/x_os/src/x_msgs/msg/X_Status.msg" NAME_WE)
add_dependencies(x_msgs_generate_messages_py _x_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(x_msgs_genpy)
add_dependencies(x_msgs_genpy x_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS x_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/x_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/x_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(x_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/x_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/x_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(x_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/x_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/x_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/x_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(x_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()

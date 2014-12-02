# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "read_imu_data: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iread_imu_data:/home/faj/catkin_ws/src/read_imu_data/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(read_imu_data_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg" NAME_WE)
add_custom_target(_read_imu_data_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "read_imu_data" "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(read_imu_data
  "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_imu_data
)

### Generating Services

### Generating Module File
_generate_module_cpp(read_imu_data
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_imu_data
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(read_imu_data_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(read_imu_data_generate_messages read_imu_data_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg" NAME_WE)
add_dependencies(read_imu_data_generate_messages_cpp _read_imu_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_imu_data_gencpp)
add_dependencies(read_imu_data_gencpp read_imu_data_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_imu_data_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(read_imu_data
  "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_imu_data
)

### Generating Services

### Generating Module File
_generate_module_lisp(read_imu_data
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_imu_data
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(read_imu_data_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(read_imu_data_generate_messages read_imu_data_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg" NAME_WE)
add_dependencies(read_imu_data_generate_messages_lisp _read_imu_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_imu_data_genlisp)
add_dependencies(read_imu_data_genlisp read_imu_data_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_imu_data_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(read_imu_data
  "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_imu_data
)

### Generating Services

### Generating Module File
_generate_module_py(read_imu_data
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_imu_data
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(read_imu_data_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(read_imu_data_generate_messages read_imu_data_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/faj/catkin_ws/src/read_imu_data/msg/IMUData.msg" NAME_WE)
add_dependencies(read_imu_data_generate_messages_py _read_imu_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_imu_data_genpy)
add_dependencies(read_imu_data_genpy read_imu_data_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_imu_data_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_imu_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_imu_data
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(read_imu_data_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_imu_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_imu_data
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(read_imu_data_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_imu_data)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_imu_data\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_imu_data
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(read_imu_data_generate_messages_py std_msgs_generate_messages_py)

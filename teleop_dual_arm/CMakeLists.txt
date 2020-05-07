cmake_minimum_required(VERSION 2.8.3)
project(teleop_dual_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # std_msgs
  # geometry_msgs
  )

### hrpsys?
# find_package(hrpsys QUIET)                                                                                        
# if(NOT ${hrpsys_FOUND})
#   find_package(PkgConfig)
#   pkg_check_modules(hrpsys hrpsys-base REQUIRED)
# endif()

# pkg_check_modules(openhrp openhrp3.1 REQUIRED)
# if (!OPENHRP_FOUND)
#   message(FATAL_ERROR  "OpenHRP not found")
# endif()

### openhrp3?
# set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
# find_package(OpenHRP REQUIRED)
# include_directories(${OPENHRP_INCLUDE_DIRS})
# link_directories(${OPENHRP_LIBRARY_DIRS})
# if(UNIX)
#   # remove -O2 ... from ${OPENHRP_DEFINITIONS}                                                                                                          
#     message(STATUS "OPENHRP_DEFINITIONS: ${OPENHRP_DEFINITIONS}")
#     message(STATUS "REMOVE -Ox option from OPENHRP_DEFINITIONS, use CMAKE_BUILD_TYPE")
#     string(REGEX REPLACE "[ ]+-O[0-9][ ]+" " " OPENHRP_DEFINITIONS "${OPENHRP_DEFINITIONS}")
#     message(STATUS "OPENHRP_DEFINITIONS: ${OPENHRP_DEFINITIONS}")
#     add_definitions(${OPENHRP_DEFINITIONS})
# endif()

### CORBA????
# find_package(PkgConfig)
# pkg_check_modules(OMNIORB REQUIRED omniORB4)
# if (!OMNIORB_FOUND)
#   message(FATAL_ERROR  "OmniORB4 is not found")
# endif()
# catkin_package(
#   DEPENDS OMNIORB
#   CATKIN_DEPENDS
# )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
)

include_directories(include
#  ${hrpsys_INCLUDE_DIRS} 
#  ${openhrp_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ik_solver src/ik_solver.cpp)

target_link_libraries(ik_solver
  # ${hrpsys_LIBRARIES}
  # ${openhrp_LIBRARIES}
  # ${OMNIORB_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

### convert from iiwa_stack/iiwa_description/uedf/iiwa14d.urdf.xacro to iiwa14d.urdf and iiwa14d.dae
execute_process(COMMAND rospack find iiwa_description OUTPUT_VARIABLE iiwa_description_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE )
execute_process(COMMAND rosrun xacro xacro ${iiwa_description_PACKAGE_PATH}/urdf/iiwa14d.urdf.xacro OUTPUT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/iiwa14d.urdf)
execute_process(COMMAND rosrun collada_urdf urdf_to_collada ${CMAKE_CURRENT_SOURCE_DIR}/iiwa14d.urdf ${CMAKE_CURRENT_SOURCE_DIR}/iiwa14d.dae)

### gen minor conf files
execute_process(COMMAND rospack find teleop_dual_arm OUTPUT_VARIABLE teleop_dual_arm_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE )
execute_process(COMMAND rospack find hrpsys_ros_bridge_tutorials OUTPUT_VARIABLE hrpsys_ros_bridge_tutorials_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE )
file(WRITE iiwa14d.conf "
model: file://${teleop_dual_arm_PACKAGE_PATH}/iiwa14d.dae
dt: 0.002
# link pose R is forcibly set with InitPose. Be careful with ee attached link init pose
end_effectors: larm,iiwa_larm_joint_ee,iiwa_torso_joint, 0.282842712,0.282842712,0, 0,0,1,0.785398163, rarm,iiwa_rarm_joint_ee,iiwa_torso_joint, 0.282842712,-0.282842712,0, 0,0,1,-0.785398163
pdgains_sim_file_name: ${teleop_dual_arm_PACKAGE_PATH}/iiwa14d.PDgain.dat
#\ SequencePlayer\ optional\ data\ (contactStates\ x\ 4\ +\ controlSwingTime\ x\ 4\ (4\ is\ lfsensor,\ rfsensor,\ lhsensor,\ rhsensor) 
seq_optional_data_dim: 4
#abc_leg_offset: 0,0.09,0
#collision_pair: RARM_WRIST_P:WAIST LARM_WRIST_P:WAIST RARM_WRIST_P:RLEG_HIP_R LARM_WRIST_P:LLEG_HIP_R RARM_WRIST_R:RLEG_HIP_R LARM_WRIST_R:LLEG_HIP_R
")
file(WRITE iiwa14d.PDgain.dat "
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
10000 1000
")
file(WRITE iiwa14d.RobotHardware.conf "
model: file://${teleop_dual_arm_PACKAGE_PATH}/iiwa14d.dae
exec_cxt.periodic.type: hrpExecutionContext
exec_cxt.periodic.rate: 500
pdgains.file_name: ${hrpsys_ros_bridge_tutorials_PACKAGE_PATH}/models/PDgains.sav
")

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


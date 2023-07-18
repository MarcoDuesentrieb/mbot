cmake_minimum_required(VERSION 3.1)
project(mbot_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# set(EXECUTABLE_OUTPUT_PATH ../files)
find_package(Threads REQUIRED)
find_package(simpleble REQUIRED CONFIG)
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs mbot_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
          INCLUDE_DIRS inc
        #  LIBRARIES beginner_tutorials
          CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}

)



add_executable(mbot_bringup src/mbot_bringup.cpp)
add_executable(mbot_controller src/mbot_controller.cpp)


target_link_libraries(mbot_bringup Threads::Threads simpleble::simpleble ${catkin_LIBRARIES})
target_link_libraries(mbot_controller Threads::Threads ${catkin_LIBRARIES})

# add_executable(find_reference_needle src/find_reference_needle.cpp)
# target_link_libraries(find_reference_needle ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(tf_scan_marker src/tf_scan_marker.cpp inc/tf_scan_marker.h)
# target_link_libraries(tf_scan_marker ${catkin_LIBRARIES})
# target_link_libraries(tf_scan_marker ${catkin_LIBRARIES} ${PCL_LIBRARIES})




################################################################################
# Install
################################################################################

install(TARGETS mbot_bringup RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS mbot_controller RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY param launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#catkin_install_python(PROGRAMS
#        scripts/stl2pcd.py
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

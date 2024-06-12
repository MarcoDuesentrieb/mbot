cmake_minimum_required(VERSION 3.1)
project(mbot_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# set(EXECUTABLE_OUTPUT_PATH ../files)
find_package(Threads REQUIRED)
find_package(simpleble REQUIRED CONFIG)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    dynamic_reconfigure
    std_msgs
    sensor_msgs
    mbot_msgs
)

generate_dynamic_reconfigure_options(
    cfg/dynrec.cfg
)


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
        #  INCLUDE_DIRS include
        #  LIBRARIES beginner_tutorials
          CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
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
add_executable(mbot_demo src/mbot_demo.cpp)

target_link_libraries(mbot_bringup Threads::Threads simpleble::simpleble ${catkin_LIBRARIES})
target_link_libraries(mbot_controller ${catkin_LIBRARIES})
target_link_libraries(mbot_demo ${catkin_LIBRARIES})



################################################################################
# Install
################################################################################

install(TARGETS mbot_bringup 
                mbot_controller
                mbot_demo
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY param launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


catkin_install_python(PROGRAMS
        scripts/mbot_battery.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

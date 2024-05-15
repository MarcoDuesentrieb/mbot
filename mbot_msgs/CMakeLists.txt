cmake_minimum_required(VERSION 2.8.3)

project(mbot_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES
    EMG.msg
    Motor.msg
)

#add_service_files(DIRECTORY srv FILES
#   Lights.srv
#)

#add_action_files(
#  DIRECTORY action FILES
#)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs)

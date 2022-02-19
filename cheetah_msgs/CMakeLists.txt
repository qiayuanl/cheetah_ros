cmake_minimum_required(VERSION 2.8.3)
project(cheetah_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        )

add_message_files(
        FILES
        LegPrefix.msg
        LegsCmd.msg
        LegsState.msg
        FeetCmd.msg
        MotorState.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
)

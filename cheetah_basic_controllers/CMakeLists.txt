cmake_minimum_required(VERSION 3.10)
project(cheetah_basic_controllers)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        pluginlib
        controller_interface
        realtime_tools
        cheetah_common
        cheetah_msgs
        dynamic_reconfigure
        tf2_ros
        )

#find_package(Boost REQUIRED COMPONENTS
#        system
#        filesystem
#        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
        cfg/Feet.cfg
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint
        pluginlib
        controller_interface
        realtime_tools
        cheetah_common
        cheetah_msgs
        dynamic_reconfigure
        tf2_ros
)

###########
## Build ##
###########

set(FLAGS
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-maybe-uninitialized
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        #        ${Boost_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/controller_base.cpp
        src/state_estimate.cpp
        src/foot_swing_trajectory.cpp
        src/feet_controller.cpp
        )

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )

target_link_libraries(${PROJECT_NAME}
        #        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

roslint_cpp()

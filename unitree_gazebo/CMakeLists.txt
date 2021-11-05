cmake_minimum_required(VERSION 3.10)
project(unitree_gazebo)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        unitree_common
        gazebo_ros
        gazebo_ros_control
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        unitree_hw_sim
        CATKIN_DEPENDS
        roscpp
        unitree_common
        gazebo_ros
        gazebo_ros_control
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(unitree_hw_sim
        src/unitree_hw_sim.cpp
        )

add_dependencies(unitree_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(unitree_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS unitree_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
#install(DIRECTORY config launch worlds
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
install(FILES unitree_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

roslint_cpp()

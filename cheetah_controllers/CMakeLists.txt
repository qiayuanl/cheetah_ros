cmake_minimum_required(VERSION 3.10)
project(cheetah_controllers)

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
        controller_interface
        unitree_control
        )

#find_package(Boost REQUIRED COMPONENTS
#        system
#        filesystem
#        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint
        controller_interface
        unitree_control
        DEPENDS
        #        Boost
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
        src/state_estimate.cpp
        src/locomotion.cpp
        )

target_link_libraries(${PROJECT_NAME}
        #        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

roslint_cpp()

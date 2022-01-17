cmake_minimum_required(VERSION 3.10)
project(cheetah_mpc_controllers)

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
        cheetah_basic_controllers
        qpoases_vendor
        trajectory_msgs
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
        cfg/Weight.cfg
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint
        controller_interface
        cheetah_basic_controllers
        qpoases_vendor
        trajectory_msgs
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
        ${pinocchio_INCLUDE_DIRS}
        ${qpoases_vendor_INCLUDE_DIRS}/qpoases_vendor
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/mpc_formulation.cpp
        src/locomotion.cpp
        )

add_dependencies(${PROJECT_NAME}
        ${PROJECT_NAME}_gencfg
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${qpoases_vendor_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

add_executable(mpc_formulation_test test/mpc_formulation_test.cpp)
target_link_libraries(mpc_formulation_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )

add_executable(mpc_solver_test test/mpc_solver_test.cpp)
target_link_libraries(mpc_solver_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )

roslint_cpp()

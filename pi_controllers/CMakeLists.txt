cmake_minimum_required(VERSION 3.10)
project(pi_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        pi_common
        controller_interface
        pi_wbc
        pi_estimation
        pi_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        dynamic_reconfigure
        rospy
        std_msgs
        message_generation
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(  
        cfg/Tutorials.cfg
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        pi_common
        controller_interface
        pi_wbc
        pi_estimation
        pi_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        dynamic_reconfigure
        rospy
        std_msgs
        message_generation
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)


## Declare a cpp library
add_library(${PROJECT_NAME}
        src/piController.cpp
        src/utilities.cpp
        )
        
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
add_executable(pi_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(pi_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(pi_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(pi_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} pi_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

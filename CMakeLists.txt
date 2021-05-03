cmake_minimum_required(VERSION 3.0.2)
project(gpg)

set(CMAKE_BUILD_TYPE Release)

# Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gpg
 CATKIN_DEPENDS roscpp rospy pcl_conversions pcl_msgs pcl_ros sensor_msgs
 #  DEPENDS system_lib
)

## Set compiler optimization flags
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare")
# set(CMAKE_CXX_FLAGS "-O3 -fopenmp -march=native -mfpmath=sse -funroll-loops -fPIC -Wno-deprecated -Wenum-compare") # no improvement
# set(CMAKE_CXX_FLAGS "-frename-registers -Ofast -march=native -fopenmp -fPIC -Wno-deprecated -Wenum-compare") # no improvement
 
# Add the headers from the <include> directory
include_directories(include)
 
# Generate the shared library from the sources
add_library(${PROJECT_NAME}_grasp_candidates_generator SHARED src/${PROJECT_NAME}/candidates_generator.cpp)

# Other libraries
add_library(${PROJECT_NAME}_antipodal src/${PROJECT_NAME}/antipodal.cpp)
add_library(${PROJECT_NAME}_cloud_camera src/${PROJECT_NAME}/cloud_camera.cpp)
add_library(${PROJECT_NAME}_config_file src/${PROJECT_NAME}/config_file.cpp)
add_library(${PROJECT_NAME}_eigen_utils src/${PROJECT_NAME}/eigen_utils.cpp)
add_library(${PROJECT_NAME}_finger_hand src/${PROJECT_NAME}/finger_hand.cpp)
add_library(${PROJECT_NAME}_frame_estimator src/${PROJECT_NAME}/frame_estimator.cpp)
add_library(${PROJECT_NAME}_grasp src/${PROJECT_NAME}/grasp.cpp)
add_library(${PROJECT_NAME}_grasp_set src/${PROJECT_NAME}/grasp_set.cpp)
add_library(${PROJECT_NAME}_hand_search src/${PROJECT_NAME}/hand_search.cpp)
add_library(${PROJECT_NAME}_local_frame src/${PROJECT_NAME}/local_frame.cpp)
add_library(${PROJECT_NAME}_plot src/${PROJECT_NAME}/plot.cpp)
add_library(${PROJECT_NAME}_point_list src/${PROJECT_NAME}/point_list.cpp)

# This executable is for testing the shared library
add_executable(${PROJECT_NAME}_generate_candidates src/generate_candidates.cpp)
target_link_libraries(${PROJECT_NAME}_generate_candidates 
                      ${PROJECT_NAME}_config_file 
                      ${PROJECT_NAME}_grasp_candidates_generator)

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_antipodal 
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_cloud_camera 
                      ${PROJECT_NAME}_eigen_utils 
                      ${PCL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_eigen_utils 
                      ${EIGEN_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_frame_estimator 
                      ${PROJECT_NAME}_cloud_camera 
                      ${PROJECT_NAME}_local_frame)

target_link_libraries(${PROJECT_NAME}_grasp 
                      ${PROJECT_NAME}_finger_hand)

target_link_libraries(${PROJECT_NAME}_grasp_set 
                      ${PROJECT_NAME}_antipodal 
                      ${PROJECT_NAME}_grasp 
                      ${PROJECT_NAME}_local_frame 
                      ${PROJECT_NAME}_point_list)

target_link_libraries(${PROJECT_NAME}_hand_search 
                      ${PROJECT_NAME}_antipodal 
                      ${PROJECT_NAME}_cloud_camera  
                      ${PROJECT_NAME}_frame_estimator 
                      ${PROJECT_NAME}_grasp_set 
                      ${PROJECT_NAME}_plot)

target_link_libraries(${PROJECT_NAME}_local_frame 
                      ${PCL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_plot 
                      ${PROJECT_NAME}_cloud_camera 
                      ${PROJECT_NAME}_grasp_set 
                      ${PROJECT_NAME}_local_frame)

target_link_libraries(${PROJECT_NAME}_grasp_candidates_generator 
                      ${PROJECT_NAME}_hand_search)
                      
target_link_libraries(${PROJECT_NAME}_point_list 
                      ${PROJECT_NAME}_eigen_utils)
                      
                      
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gpg
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_integration src/ros_integration.cpp)
add_dependencies(ros_integration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_integration
${catkin_LIBRARIES}
${PROJECT_NAME}_config_file
${PROJECT_NAME}_grasp_candidates_generator)
         
# Rename executable.
set_target_properties(${PROJECT_NAME}_generate_candidates 
                      PROPERTIES OUTPUT_NAME generate_candidates 
                      PREFIX "")

set_target_properties(${PROJECT_NAME}_grasp_candidates_generator 
                      PROPERTIES OUTPUT_NAME grasp_candidates_generator)




# Set the location for library installation (/usr/lib in this case is not really necessary)
# Use "sudo make install" to apply
install(TARGETS ${PROJECT_NAME}_grasp_candidates_generator 
        DESTINATION lib)

install(DIRECTORY include/gpg DESTINATION include)

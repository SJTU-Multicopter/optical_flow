cmake_minimum_required(VERSION 2.8.3)
project(optical_flow)

find_package(catkin REQUIRED COMPONENTS  
 roscpp  
 rospy  
 std_msgs  
 sensor_msgs  
 cv_bridge  
 image_transport
 message_generation  
) 

find_package(OpenCV REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES kalman_filter
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(  
include  
${OpenCV_INCLUDE_DIRS}  
${catkin_INCLUDE_DIRS}  
)  


#add_executable(test1 src/test.cpp)
#target_link_libraries(test1
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#)


cmake_minimum_required(VERSION 2.8.3)
project(object_detector)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
set( LIBRARY_PATH "${PROJECT_SOURCE_DIR}/lib/${ARCHITECTURE}" )
set( GL_LIBRARY "${LIBRARY_PATH}/libobject_draw.so" "${LIBRARY_PATH}/libobject_detector.so" "${LIBRARY_PATH}/libgl_driver.so" "${LIBRARY_PATH}/libserial.so" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GL_LIBRARY}
)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${GL_LIBRARY}
)

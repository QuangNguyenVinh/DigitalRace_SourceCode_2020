cmake_minimum_required(VERSION 2.8.3)
project(cdsntu)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_DIR "/usr/local/share/OpenCV/")
find_package(  OpenCV 3.4.3 REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS Boost OpenCV
)

include_directories(
  include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/DetectLane.cpp
        src/ControlCar.cpp
        src/DetectSign.cpp
        src/DetectObstacle.cpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)



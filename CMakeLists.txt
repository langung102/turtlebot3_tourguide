cmake_minimum_required(VERSION 3.9)
project(turtlebot3_tourguide)

#build turltebot3_firebase lib
link_directories(/usr/lib/firebase /usr/lib/firebase/deps/app /usr/lib/firebase/deps/database/external /usr/lib/aarch64-linux-gnu)

set(firebase_libs 
    firebase_firestore
    firebase_storage
    firebase_functions
    firebase_database
    firebase_app
    firebase_rest_lib
    leveldb
    z
    flatbuffers
    libuWS
    secret-1
    glib-2.0
    ssl
    crypto
    curl
)

include_directories(inc)

set(SRC_LIB src/turtlebot3_firebase.cpp)

add_library(turtlebot3_firebase ${SRC_LIB})
target_link_libraries(turtlebot3_firebase "${firebase_libs}")

#build turltebot3_tourguide

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_action REQUIRED)

set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=1")

set(SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/request_handler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/update_handler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/global.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/request_handler.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/update_handler.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/navigation.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/inc/global.hpp
)

add_executable(turtlebot3_tourguide
  ${SRC}
)
target_link_libraries(turtlebot3_tourguide "${firebase_libs}" ${CMAKE_BINARY_DIR}/libturtlebot3_firebase.a)
add_dependencies(turtlebot3_tourguide turtlebot3_firebase)

ament_target_dependencies(turtlebot3_tourguide
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  rclcpp_action  
  nav2_lifecycle_manager
  tf2_geometry_msgs
  sensor_msgs
  nav2_util
)

install(TARGETS
turtlebot3_tourguide
  DESTINATION lib/${PROJECT_NAME})

ament_package()
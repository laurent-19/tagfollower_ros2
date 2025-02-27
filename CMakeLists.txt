cmake_minimum_required(VERSION 3.8)
project(tagfollower_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui video)
find_package(sensor_msgs REQUIRED)

# Find message generation packages
find_package(rosidl_default_generators REQUIRED)

# Define message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Matrix33.msg"
  "msg/NavdataAdcDataFrame.msg"
  "msg/NavdataAltitude.msg"
  "msg/NavdataDemo.msg"
  "msg/NavdataEulerAngles.msg"
  "msg/NavdataGames.msg"
  "msg/NavdataGyrosOffsets.msg"
  "msg/NavdataHdvideoStream.msg"
  "msg/NavdataKalmanPressure.msg"
  "msg/NavdataMagneto.msg"
  "msg/Navdata.msg"
  "msg/NavdataPhysMeasures.msg"
  "msg/NavdataPressureRaw.msg"
  "msg/NavdataPwm.msg"
  "msg/NavdataRawMeasures.msg"
  "msg/NavdataRcReferences.msg"
  "msg/NavdataReferences.msg"
  "msg/NavdataTime.msg"
  "msg/NavdataTrackersSend.msg"
  "msg/NavdataTrims.msg"
  "msg/NavdataVideoStream.msg"
  "msg/NavdataVisionDetect.msg"
  "msg/NavdataVision.msg"
  "msg/NavdataVisionOf.msg"
  "msg/NavdataVisionPerf.msg"
  "msg/NavdataVisionRaw.msg"
  "msg/NavdataWatchdog.msg"
  "msg/NavdataWifi.msg"
  "msg/NavdataWindSpeed.msg"
  "msg/NavdataZimmu3000.msg"
  "msg/Vector21.msg"
  "msg/Vector31.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Generate the typesupport target once
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Create a separate library to link against message interfaces
add_library(${PROJECT_NAME}_node SHARED
  src/followControl_Original.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Make sure the library is built after the messages are generated
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})

# Link dependencies for the library
target_link_libraries(${PROJECT_NAME}_node
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

# Declare executable
add_executable(tagfollower_node src/followControl_Original.cpp)

# Link the executable
target_link_libraries(tagfollower_node 
  ${PROJECT_NAME}_node
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  tagfollower_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install message files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
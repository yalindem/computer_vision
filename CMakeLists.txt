cmake_minimum_required(VERSION 3.8)
project(computer_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Erosion.srv"
  DEPENDENCIES sensor_msgs
)
ament_package()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(image_blur_node src/image_blur_node.cpp)
ament_target_dependencies(image_blur_node rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(image_blur_node ${OpenCV_LIBRARIES})

add_executable(static_image_publisher_node src/static_image_publisher_node.cpp)
ament_target_dependencies(static_image_publisher_node rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(static_image_publisher_node ${OpenCV_LIBRARIES})

add_executable(image_edge_detection_node src/image_edge_detection_node.cpp)
ament_target_dependencies(image_edge_detection_node rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(image_edge_detection_node ${OpenCV_LIBRARIES})

add_executable(image_erosion_server_service src/image_erosion_server_service.cpp)
ament_target_dependencies(image_erosion_server_service rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(image_erosion_server_service ${OpenCV_LIBRARIES})

# Otomatik üretilen servis arayüzlerini hedefe bağla:
rosidl_target_interfaces(image_erosion_server_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  static_image_publisher_node
  image_blur_node
  image_edge_detection_node
  image_erosion_server_service
  DESTINATION lib/${PROJECT_NAME}
)


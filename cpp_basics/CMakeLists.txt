cmake_minimum_required(VERSION 3.8)
project(cpp_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wego_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(calculator REQUIRED)
find_package(rclcpp_components REQUIRED)

# make executable
add_executable(talker ./src/topic_publisher.cpp)
ament_target_dependencies(talker rclcpp wego_msgs)

add_executable(listener ./src/topic_subscriber.cpp)
ament_target_dependencies(listener rclcpp wego_msgs)

add_executable(use_plugin ./src/use_plugin.cpp)
ament_target_dependencies(use_plugin pluginlib calculator)

# 파일을 설치할 때 target이 설치될 위치 설정
install(
  TARGETS talker listener use_plugin
  DESTINATION lib/${PROJECT_NAME} 
)

# composable node
add_library(topic_publisher_component SHARED src/topic_publisher_comp.cpp)
ament_target_dependencies(topic_publisher_component rclcpp wego_msgs rclcpp_components)
target_compile_definitions(topic_publisher_component PRIVATE "BUILDING_DLL")

add_library(topic_subscriber_component SHARED src/topic_subscriber_comp.cpp)
ament_target_dependencies(topic_subscriber_component rclcpp wego_msgs rclcpp_components)
target_compile_definitions(topic_subscriber_component PRIVATE "BUILDING_DLL")

rclcpp_components_register_node(
    topic_publisher_component
    PLUGIN "wego::TopicPublisher"
    EXECUTABLE topic_publisher
)

rclcpp_components_register_node(
    topic_subscriber_component
    PLUGIN "wego::TopicSubscriber"
    EXECUTABLE topic_subscriber
)

install(TARGETS 
        topic_publisher_component 
        topic_subscriber_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
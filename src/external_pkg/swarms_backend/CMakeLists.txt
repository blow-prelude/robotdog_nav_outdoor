cmake_minimum_required(VERSION 3.5)
project(backend_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++17：thing_json（结构化绑定等）与 Ubuntu 20.04 默认 GCC 9+ / RK3588 常用工具链一致
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEBUG or RELEASE
set(CMAKE_BUILD_TYPE $ENV{CMAKE_BUILD_TYPE})
if(${CMAKE_BUILD_TYPE} MATCHES "RELEASE")
  message(${PROJECT_NAME} ": RELEASE MODE")
  add_compile_definitions(NDEBUG)
else()
  message(${PROJECT_NAME} ": DEBUG MODE")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(msg_set_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)

# include dirs
include_directories(
        SYSTEM
        #/usr/include/jsoncpp
        ${YAML_CPP_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/third_party
)

#link_directories(
 #       /usr/local/lib
#)

add_executable(backend_nodes_launcher
        src/main.cpp
        src/backend_ros/backend_node.cpp
        src/mqttnode/mqttnode.cpp
        src/thing_gateway/product_mqtt_topics.cpp
        src/thing_gateway/thing_json/parse.cpp
        src/thing_gateway/thing_json/serialize.cpp
        src/thing_gateway/product_thing_services_dispatcher.cpp
        src/thing_gateway/product_mqtt_topic_router.cpp
        src/thing_gateway/product_mqtt_ros_bridge.cpp
        src/thing_gateway/product_gateway_node.cpp
        src/thing_gateway/handlers/services_alert_task.cpp
        src/thing_gateway/handlers/services_perception.cpp
        src/thing_gateway/handlers/services_payload_control.cpp
        src/mavlink_ros/serial_port.cpp
        src/mavlink_ros/udp_port.cpp
        src/mavlink_ros/mavlink_ros_interface.cpp
        src/mavlink_ros/mavlink_ros_node.cpp
        )

ament_target_dependencies(backend_nodes_launcher
        rclcpp
        msg_set_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        mavros_msgs
        tf2
        #jsoncpp
        PahoMqttCpp
        ament_index_cpp
        )

# link
target_link_libraries(backend_nodes_launcher
        jsoncpp_lib
        #nlohmann_json::nlohmann_json
        paho-mqtt3as
        paho-mqttpp3
        yaml-cpp
        ${OPENSSL_LIBRARIES}
        )

# install
install(TARGETS  backend_nodes_launcher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY  launch  DESTINATION share/${PROJECT_NAME})

# TEST
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

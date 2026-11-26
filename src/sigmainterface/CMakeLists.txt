cmake_minimum_required(VERSION 3.10)

project(sigma2ros_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 导出 compile_commands.json，供 clangd/IDE 解析各目标的 include 路径
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# sdk 源码直接编入可执行，不产独立库
set(UTILS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/utils" CACHE PATH "header-only 工具目录")
set(SIGMA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sigma_sdk" CACHE PATH "SigmaSDK SDK目录")

# 消息定义（全 ENU 米/弧度；EntityId 为嵌套标识，被依赖须列在最前）
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/EntityId.msg"
    "msg/ModelInfo.msg"
    "msg/TargetFusion.msg"
    "msg/MotionCommand.msg"
    "msg/SpawnModel.msg"
    "msg/ReferenceTrajectoryPoint.msg"
    "msg/ReferenceTrajectory.msg"
    "msg/VelocityCommand.msg"
    "msg/WeaponFireCommand.msg"
    "msg/WeaponFireAck.msg"
    "msg/WeaponSpec.msg"
    "msg/Loadout.msg"
    "srv/LoadoutQuery.srv")

add_executable(sigma2ros_bridge_node
    src/sigma2ros_bridge.cpp
    src/home_state.cpp
    src/entity_topics.cpp
    ${SIGMA_SDK_DIR}/sigmaSDK.cpp)

target_include_directories(sigma2ros_bridge_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${UTILS_DIR} ${SIGMA_SDK_DIR})

rosidl_get_typesupport_target(sigma2ros_msgs_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(sigma2ros_bridge_node PRIVATE
    rclcpp::rclcpp ${sigma2ros_msgs_target} Threads::Threads)

# launch 脚本与参数/home 配置随包装到 share，供 ros2 launch 与部署使用
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS sigma2ros_bridge_node
        DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
    enable_testing()
    add_executable(velocity_wire_contract test/velocity_wire_contract.cpp)
    target_include_directories(velocity_wire_contract PRIVATE ${SIGMA_SDK_DIR})
    add_test(NAME velocity_wire_contract COMMAND velocity_wire_contract)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()

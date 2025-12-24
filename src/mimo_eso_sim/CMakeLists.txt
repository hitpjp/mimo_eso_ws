cmake_minimum_required(VERSION 3.8)
project(mimo_eso_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED) # 必须包含 Eigen

# 2. 包含头文件目录
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 3. 生成物理仿真节点 (Plant)
add_executable(plant_node src/plant_node.cpp)
ament_target_dependencies(plant_node rclcpp geometry_msgs)
# 如果 plant_node 用到了 Eigen，也需要链接
target_link_libraries(plant_node Eigen3::Eigen)

# 4. 生成观测器节点 (ESO)
add_executable(eso_node src/eso_node.cpp)
ament_target_dependencies(eso_node rclcpp geometry_msgs)
target_link_libraries(eso_node Eigen3::Eigen)

# 5. 安装路径（可选，但在 colcon build 后运行需要）
install(TARGETS
  plant_node
  eso_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(megatron_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # Required for custom msg generation
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# acados

set(ACADOS_INSTALL_DIR "/home/user/6G/acados_ws/acados/")
set(MEGATRON_ACADOS_C_GENERATED_FILES ${PROJECT_SOURCE_DIR}/acados_mpc/c_generated_code)


include_directories(
  ${MEGATRON_ACADOS_C_GENERATED_FILES}
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/acados/
  ${ACADOS_INSTALL_DIR}/include/acados_c
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
  ${ACADOS_INSTALL_DIR}/include/qpOASES_e/
)

add_executable(mpc_controller_node src/mpc_controller_node.cpp)

ament_target_dependencies(mpc_controller_node
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
)

target_include_directories(mpc_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mpc_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(mpc_controller_node 
  ${MEGATRON_ACADOS_C_GENERATED_FILES}/libacados_ocp_solver_megatron.so
  ${ACADOS_INSTALL_DIR}/lib/libacados.so
)


add_executable(mpc_lifecycle_client_node src/mpc_lifecycle_client.cpp)

ament_target_dependencies(mpc_lifecycle_client_node
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs
)

add_executable(robot_model_sim_node src/robot_model_sim.cpp)

ament_target_dependencies(robot_model_sim_node 
  rclcpp 
  geometry_msgs
  nav_msgs 
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS 
  mpc_controller_node
  mpc_lifecycle_client_node
  robot_model_sim_node
  DESTINATION lib/${PROJECT_NAME})


install(FILES data/reference_traj_kinematics.csv
        DESTINATION share/${PROJECT_NAME}/data)

install(FILES
  ${MEGATRON_ACADOS_C_GENERATED_FILES}/libacados_ocp_solver_megatron.so
  ${ACADOS_INSTALL_DIR}/lib/libacados.so
  DESTINATION lib
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()


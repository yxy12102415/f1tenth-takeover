cmake_minimum_required(VERSION 3.8)
project(takeover_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

if(NOT DEFINED ENV{ACADOS_INSTALL_DIR})
  message(FATAL_ERROR "ACADOS_INSTALL_DIR must point to acados v0.4.0")
endif()
file(REAL_PATH "$ENV{ACADOS_INSTALL_DIR}" ACADOS_PREFIX)
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(V8_ROOT "${REPO_ROOT}/real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8")
set(GENERATED "${V8_ROOT}/generated_joint")

file(GLOB MODEL_SOURCES
  "${GENERATED}/joint_vehicle_model/*.c"
  "${GENERATED}/joint_vehicle_cost/*.c"
  "${GENERATED}/joint_vehicle_constraints/*.c")

add_library(takeover_joint_solver SHARED
  "${GENERATED}/acados_solver_joint_vehicle.c"
  ${MODEL_SOURCES})
target_include_directories(takeover_joint_solver PRIVATE
  "${ACADOS_PREFIX}/include"
  "${ACADOS_PREFIX}/include/blasfeo/include"
  "${ACADOS_PREFIX}/include/hpipm/include"
  "${GENERATED}")
target_link_directories(takeover_joint_solver PRIVATE "${ACADOS_PREFIX}/lib")
target_link_libraries(takeover_joint_solver acados hpipm blasfeo m)
set_target_properties(takeover_joint_solver PROPERTIES
  BUILD_RPATH "${ACADOS_PREFIX}/lib"
  INSTALL_RPATH "${ACADOS_PREFIX}/lib")

add_executable(takeover_controller_node
  src/takeover_controller_node.cpp
  "${V8_ROOT}/src/dynamic_mpc.cpp"
  "${V8_ROOT}/src/mpc_iteration.cpp")
target_include_directories(takeover_controller_node PRIVATE
  include
  "${V8_ROOT}/src/cpp_frenet"
  "${GENERATED}"
  "${ACADOS_PREFIX}/include"
  "${ACADOS_PREFIX}/include/blasfeo/include"
  "${ACADOS_PREFIX}/include/hpipm/include"
  ${JSONCPP_INCLUDE_DIRS})
target_link_directories(takeover_controller_node PRIVATE "${ACADOS_PREFIX}/lib")
target_link_libraries(takeover_controller_node
  takeover_joint_solver acados hpipm blasfeo Eigen3::Eigen ${JSONCPP_LIBRARIES})
ament_target_dependencies(takeover_controller_node
  rclcpp ackermann_msgs nav_msgs std_msgs geometry_msgs)
set_target_properties(takeover_controller_node PROPERTIES
  BUILD_RPATH "${ACADOS_PREFIX}/lib"
  INSTALL_RPATH "${ACADOS_PREFIX}/lib")

install(TARGETS takeover_joint_solver takeover_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

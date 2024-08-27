cmake_minimum_required(VERSION 3.29)
project(PinocchioExample)

# 设置使用C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 找到Pinocchio和Eigen3
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置hpp-fcl的自定义路径
find_library(HPP_FCL_LIBRARY NAMES hpp-fcl HINTS /opt/openrobots/lib NO_DEFAULT_PATH)
find_path(HPP_FCL_INCLUDE_DIR hpp/fcl/collision_object.h HINTS /opt/openrobots/include)

if(HPP_FCL_LIBRARY AND HPP_FCL_INCLUDE_DIR)
  include_directories(${HPP_FCL_INCLUDE_DIR})
  link_libraries(${HPP_FCL_LIBRARY})
else()
  message(FATAL_ERROR "hpp-fcl library or include directory not found in /opt/openrobots/")
endif()

message(STATUS "HPP_FCL_LIBRARY: ${HPP_FCL_LIBRARY}")

# 找到 OMPL 库
find_package(ompl REQUIRED)

# 找到 Boost 库
find_package(Boost COMPONENTS filesystem serialization system REQUIRED)

# 查找 urdfdom 相关库
find_library(URDFDOM_MODEL_LIB urdfdom_model REQUIRED)
find_library(URDFDOM_MODEL_STATE_LIB urdfdom_model_state REQUIRED)
find_library(URDFDOM_SENSOR_LIB urdfdom_sensor REQUIRED)
find_library(URDFDOM_WORLD_LIB urdfdom_world REQUIRED)

# 包含头文件目录
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)

# 链接 Boost 库
link_libraries(${Boost_LIBRARIES})

# 添加子目录
add_subdirectory(IK)
add_subdirectory(collisionDetection)
add_subdirectory(planning)
add_subdirectory(math)

# 编译选项
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-ignored-attributes)
endif()

# 创建一个可执行文件
add_executable(collisionDetection collisionDetection.cpp)

# 链接库
target_link_libraries(collisionDetection
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${HPP_FCL_LIB}
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${URDFDOM_MODEL_LIB}
  ${URDFDOM_MODEL_STATE_LIB}
  ${URDFDOM_SENSOR_LIB}
  ${URDFDOM_WORLD_LIB}
)

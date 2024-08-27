# 创建一个可执行文件
add_executable(pinoIK pinoIK.cpp)

# 链接库
target_link_libraries(pinoIK
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

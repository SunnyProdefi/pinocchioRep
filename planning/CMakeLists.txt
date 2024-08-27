include_directories(${CMAKE_SOURCE_DIR})
add_executable(dualArmRRT dualArmRRT.cpp ../robotModel/robot_model.cpp ../math/BSplineInterpolator.cpp)
# add_executable(dualArmRRT dualArmRRT.cpp ../robotModel/robot_model.cpp)
target_link_libraries(dualArmRRT 
    ${OMPL_LIBRARIES}  
    ${pinocchio_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${HPP_FCL_LIB}
)
add_compile_options(-Wall -Wextra)
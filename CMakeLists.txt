cmake_minimum_required(VERSION 3.0.2)
project(modular_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    open_manipulator_msgs
)

# == NUEVO: Incluir ONNX Runtime ==
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/scf-usuario2/onnxruntime-linux-x64-1.17.1/include  # <<< Agrega include de ONNX
)

link_directories(
  /home/scf-usuario2/onnxruntime-linux-x64-1.17.1/lib  # <<< Agrega lib de ONNX
)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs open_manipulator_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(modular_controller src/modular_controller.cpp)
add_executable(modular_controller_twoHands src/modular_controller_twoHands.cpp)
add_executable(hand_pose_test src/hand_pose_test.cpp)
add_executable(gripper_test src/gripper_test.cpp)
add_executable(servos_test src/servos_test.cpp)

add_dependencies(modular_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(modular_controller_twoHands ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hand_pose_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gripper_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(servos_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(modular_controller ${catkin_LIBRARIES} onnxruntime)
target_link_libraries(modular_controller_twoHands ${catkin_LIBRARIES} onnxruntime)
target_link_libraries(hand_pose_test ${catkin_LIBRARIES} onnxruntime )
target_link_libraries(gripper_test ${catkin_LIBRARIES} )
target_link_libraries(servos_test ${catkin_LIBRARIES} )




################################################################################
# Install
################################################################################
install(TARGETS modular_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################


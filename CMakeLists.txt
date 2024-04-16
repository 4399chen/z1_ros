cmake_minimum_required(VERSION 3.0.2)
project(z1_ros)

# 编译为 C++14，因原始 SDK 使用了 C++14
add_compile_options(-std=c++14)

# 寻找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

# 寻找OpenCV
find_package(OpenCV REQUIRED)

# 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 从原始SDK复制
link_directories(lib)

# 设置示例文件路径
set(EXAMPLES_FILES
    examples/highcmd_basic.cpp
    examples/highcmd_development.cpp
    examples/lowcmd_development.cpp
    examples/lowcmd_multirobots.cpp
    src/highcmd_basic_ros.cpp
    src/eye_in_hand_calibration.cpp
    src/eye_to_hand_calibration.cpp
    src/test_eye_in_hand_calibration.cpp
    src/test_eye_to_hand_calibration.cpp
)

# 对于每个示例文件，创建一个可执行文件并链接库
foreach(EXAMPLE_FILE IN LISTS EXAMPLES_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} ${catkin_LIBRARIES} Z1_SDK_${CMAKE_SYSTEM_PROCESSOR} ${OpenCV_LIBS})
endforeach()


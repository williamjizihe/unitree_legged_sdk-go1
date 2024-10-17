cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

# 检查架构和操作系统
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

# 查找 OpenCV
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

# 包含头文件和库文件目录
link_directories(${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Python 构建选项
option(PYTHON_BUILD "build python wrapper" OFF)
if(PYTHON_BUILD)
  add_subdirectory(python_wrapper)
endif()

# 设置编译选项
set(EXTRA_LIBS -pthread udev libunitree_legged_sdk.a unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)

# Catkin 包支持
find_package(catkin QUIET)
if(${catkin_FOUND})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  )
endif()

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# 编译示例程序
add_executable(example_position example/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity example/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque example/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk example/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_joystick example/example_joystick.cpp)
target_link_libraries(example_joystick ${EXTRA_LIBS})

# 编译并链接 OpenCV 库
add_executable(example_getRectFrame example/example_getRectFrame.cpp)
target_link_libraries(example_getRectFrame ${EXTRA_LIBS} ${OpenCV_LIBS})

add_executable(example_navigation example/example_navigation.cpp)
target_link_libraries(example_navigation ${EXTRA_LIBS} ${OpenCV_LIBS})

# 安装目标
install(TARGETS
  example_position example_velocity example_torque example_walk example_joystick example_getRectFrame
  DESTINATION bin/unitree)

install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/unitree
  USE_SOURCE_PERMISSIONS)

cmake_minimum_required(VERSION 3.8)
project(pj_lidar)

# Стандарты C/C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск ROS-зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Поиск libserial (системная библиотека)
find_path(LIBSERIAL_INCLUDE_DIR
  NAMES libserial/SerialPort.h
  PATHS /usr/include /usr/local/include
  NO_DEFAULT_PATH
)

find_library(LIBSERIAL_LIBRARY
  NAMES serial
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
  NO_DEFAULT_PATH
)

if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "libserial not found! Please install: sudo apt install libserial-dev")
else()
  message(STATUS "Found libserial include: ${LIBSERIAL_INCLUDE_DIR}")
  message(STATUS "Found libserial library: ${LIBSERIAL_LIBRARY}")
endif()

# Исполняемый файл
add_executable(lidar_processor
  src/lidar_processor.cpp
  c++/pj_lidar/lidar_data_new.h
  c++/pj_lidar/udp_handler_new.h
)

# ROS-зависимости
ament_target_dependencies(lidar_processor
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  tf2_ros
)

# Добавляем пути к заголовкам libserial и линкуем библиотеку
target_include_directories(lidar_processor PRIVATE ${LIBSERIAL_INCLUDE_DIR})
target_link_libraries(lidar_processor ${LIBSERIAL_LIBRARY})

# Установка
install(TARGETS lidar_processor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY python/pj_lidar DESTINATION share/${PROJECT_NAME}/python)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY python/pj_lidar/config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES python/pj_lidar/config.yaml DESTINATION share/${PROJECT_NAME})

# Python executables для ROS2 launch - копируем напрямую вместо symlink
install(PROGRAMS 
  python/pj_lidar/esp32_tcp_client.py 
  python/pj_lidar/lidar_udp_server_new.py 
  python/pj_lidar/scan_state_machine.py 
  python/pj_lidar/robot_monitor.py 
  python/pj_lidar/lidar_motor_controller.py
  python/pj_lidar/static_tf_broadcaster.py
  python/pj_lidar/slam_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/doc)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
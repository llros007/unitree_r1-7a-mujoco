cmake_minimum_required(VERSION 3.10)
project(motor_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(motor_controller SHARED
  src/MotorController.cpp
  src/MotorProtocol.cpp
  src/SerialPort.cpp
)

target_include_directories(motor_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(motor_controller
  PRIVATE MOTOR_CONTROLLER_SHARED MOTOR_CONTROLLER_BUILDING
  INTERFACE MOTOR_CONTROLLER_SHARED
)

target_link_libraries(motor_controller PUBLIC Threads::Threads)

if(WIN32)
  target_link_libraries(motor_controller PUBLIC ws2_32)
endif()

set_target_properties(motor_controller PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 0
  OUTPUT_NAME motor_controller
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

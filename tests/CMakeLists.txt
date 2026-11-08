cmake_minimum_required(VERSION 3.20)
project(DMMotorProtocolTests LANGUAGES CXX)

set(LIBXR_SOURCE_DIR "" CACHE PATH "LibXR checkout")
set(MOTOR_MODULE_DIR "" CACHE PATH "QDU-Robomaster/Motor checkout")
if(NOT EXISTS "${LIBXR_SOURCE_DIR}/src/driver/can.cpp"
   OR NOT EXISTS "${MOTOR_MODULE_DIR}/Motor.hpp")
  message(FATAL_ERROR "Set LIBXR_SOURCE_DIR and MOTOR_MODULE_DIR to their checkouts")
endif()

# Use the actual LibXR CAN dispatcher and mailbox; only the bus and clock are simulated.
add_executable(dmmotor_protocol
  protocol.cpp
  "${LIBXR_SOURCE_DIR}/src/driver/can.cpp"
  "${LIBXR_SOURCE_DIR}/src/structure/lockfree_list.cpp")
target_compile_features(dmmotor_protocol PRIVATE cxx_std_20)
target_compile_definitions(dmmotor_protocol PRIVATE
  LIBXR_SINGLE_CORE=1 LIBXR_DEFAULT_SCALAR=double XR_LOG_MESSAGE_MAX_LEN=64)
target_include_directories(dmmotor_protocol PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/.."
  "${MOTOR_MODULE_DIR}"
  "${LIBXR_SOURCE_DIR}/system/none"
  "${LIBXR_SOURCE_DIR}/lib/Eigen"
  "${LIBXR_SOURCE_DIR}/src"
  "${LIBXR_SOURCE_DIR}/src/core"
  "${LIBXR_SOURCE_DIR}/src/system"
  "${LIBXR_SOURCE_DIR}/src/utils"
  "${LIBXR_SOURCE_DIR}/src/structure"
  "${LIBXR_SOURCE_DIR}/src/structure/queue"
  "${LIBXR_SOURCE_DIR}/src/middleware"
  "${LIBXR_SOURCE_DIR}/src/driver")
file(GLOB_RECURSE LIBXR_HEADERS CONFIGURE_DEPENDS "${LIBXR_SOURCE_DIR}/src/*.hpp")
set(LIBXR_INCLUDE_DIRS "")
foreach(header IN LISTS LIBXR_HEADERS)
  get_filename_component(header_dir "${header}" DIRECTORY)
  list(APPEND LIBXR_INCLUDE_DIRS "${header_dir}")
endforeach()
list(REMOVE_DUPLICATES LIBXR_INCLUDE_DIRS)
target_include_directories(dmmotor_protocol PRIVATE ${LIBXR_INCLUDE_DIRS})
if(MSVC)
  target_compile_options(dmmotor_protocol PRIVATE /utf-8)
endif()
enable_testing()
add_test(NAME dmmotor_protocol COMMAND dmmotor_protocol)

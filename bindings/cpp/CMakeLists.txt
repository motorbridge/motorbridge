cmake_minimum_required(VERSION 3.16)
project(motorbridge_cpp VERSION 0.5.3 LANGUAGES CXX)

include(CMakePackageConfigHelpers)

set(
  MOTORBRIDGE_ABI_HEADER
  "${CMAKE_CURRENT_LIST_DIR}/../../motor_abi/include/motor_abi.h"
  CACHE FILEPATH
  "Path to motor_abi.h for installation."
)

add_library(motorbridge_cpp INTERFACE)
add_library(motorbridge::cpp ALIAS motorbridge_cpp)
set_target_properties(motorbridge_cpp PROPERTIES EXPORT_NAME cpp)

target_compile_features(motorbridge_cpp INTERFACE cxx_std_17)
target_include_directories(
  motorbridge_cpp
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../motor_abi/include>
    $<INSTALL_INTERFACE:include>
)

option(MOTORBRIDGE_CPP_BUILD_EXAMPLES "Build C++ wrapper examples." ON)
set(
  MOTORBRIDGE_ABI_LIBRARY
  ""
  CACHE FILEPATH
  "Optional path to motor_abi shared/static library for local example linking."
)

if(MOTORBRIDGE_CPP_BUILD_EXAMPLES)
  if(MOTORBRIDGE_ABI_LIBRARY STREQUAL "")
    message(STATUS "MOTORBRIDGE_ABI_LIBRARY not set; skip cpp_wrapper_demo target.")
  else()
    add_executable(cpp_wrapper_demo examples/cpp_wrapper_demo.cpp)
    target_link_libraries(cpp_wrapper_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(robstride_wrapper_demo examples/robstride_wrapper_demo.cpp)
    target_link_libraries(robstride_wrapper_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(hexfellow_canfd_demo examples/hexfellow_canfd_demo.cpp)
    target_link_libraries(hexfellow_canfd_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(scan_ids_demo examples/scan_ids_demo.cpp)
    target_link_libraries(scan_ids_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(pos_ctrl_demo examples/pos_ctrl_demo.cpp)
    target_link_libraries(pos_ctrl_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(pos_repl_demo examples/pos_repl_demo.cpp)
    target_link_libraries(pos_repl_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(full_modes_demo examples/full_modes_demo.cpp)
    target_link_libraries(full_modes_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")

    add_executable(pid_register_tune_demo examples/pid_register_tune_demo.cpp)
    target_link_libraries(pid_register_tune_demo PRIVATE motorbridge::cpp "${MOTORBRIDGE_ABI_LIBRARY}")
  endif()
endif()

install(TARGETS motorbridge_cpp EXPORT motorbridgeTargets)
install(DIRECTORY include/ DESTINATION include)

if(EXISTS "${MOTORBRIDGE_ABI_HEADER}")
  install(FILES "${MOTORBRIDGE_ABI_HEADER}" DESTINATION include)
endif()

install(
  EXPORT motorbridgeTargets
  NAMESPACE motorbridge::
  FILE motorbridgeTargets.cmake
  DESTINATION lib/cmake/motorbridge
)

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/motorbridge-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/motorbridge-config.cmake"
  INSTALL_DESTINATION lib/cmake/motorbridge
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/motorbridge-config-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/motorbridge-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/motorbridge-config-version.cmake"
  DESTINATION lib/cmake/motorbridge
)

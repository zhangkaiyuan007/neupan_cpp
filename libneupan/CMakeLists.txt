cmake_minimum_required(VERSION 3.16)
project(libneupan VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(NEUPAN_NATIVE "Enable -march=native tuning" OFF)
option(NEUPAN_BUILD_TESTS "Build unit tests" ON)
option(NEUPAN_BUILD_PYTHON "Build the pybind11 Python module" OFF)

find_package(Eigen3 3.3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(NEUPAN_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(NEUPAN_YAML_CPP_TARGET yaml-cpp)
endif()

if(NEUPAN_NATIVE)
  message(WARNING "NEUPAN_NATIVE requires an osqp-eigen built with the same "
                  "-march flags; use with care.")
  add_compile_options(-march=native -mtune=native)
endif()

add_library(neupan
  src/mlp.cpp
  src/robot.cpp
  src/dune.cpp
  src/nrmp.cpp
  src/initial_path.cpp
  src/pan.cpp
  src/neupan_planner.cpp
  src/tensor_io.cpp
)
add_library(neupan::neupan ALIAS neupan)

target_include_directories(neupan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(neupan PUBLIC Eigen3::Eigen OsqpEigen::OsqpEigen
                                    ${NEUPAN_YAML_CPP_TARGET})
target_compile_options(neupan PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/O2>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-O3>)

if(NEUPAN_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)

  foreach(t test_robot test_dune test_nrmp test_replay)
    add_executable(${t} tests/${t}.cpp)
    target_link_libraries(${t} PRIVATE neupan GTest::gtest GTest::gtest_main)
    target_compile_definitions(${t} PRIVATE
      NEUPAN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
      NEUPAN_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

if(NEUPAN_BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(neupan_cpp_py python/bindings.cpp)
  target_link_libraries(neupan_cpp_py PRIVATE neupan)
  set_target_properties(neupan_cpp_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python")
endif()

add_executable(neupan_bench tools/bench.cpp)
target_link_libraries(neupan_bench PRIVATE neupan)
target_compile_definitions(neupan_bench PRIVATE
  NEUPAN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
  NEUPAN_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")

install(TARGETS neupan EXPORT neupanTargets)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/ DESTINATION share/libneupan/models)

install(FILES package.xml DESTINATION share/libneupan)

install(EXPORT neupanTargets
  FILE libneupanTargets.cmake
  NAMESPACE neupan::
  DESTINATION lib/cmake/libneupan)

# Export the installed models directory so dependents (e.g. neupan_cpp_ros)
# can vendor the DUNE model into their own share.
include(CMakePackageConfigHelpers)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/libneupanConfig.cmake" "\
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(OsqpEigen)
find_dependency(yaml-cpp)
get_filename_component(libneupan_MODELS_DIR
  \"\${CMAKE_CURRENT_LIST_DIR}/../../../share/libneupan/models\" ABSOLUTE)
include(\"\${CMAKE_CURRENT_LIST_DIR}/libneupanTargets.cmake\")
")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libneupanConfig.cmake"
  DESTINATION lib/cmake/libneupan)

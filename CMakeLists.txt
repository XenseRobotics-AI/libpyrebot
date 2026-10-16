cmake_minimum_required(VERSION 3.18)
project(libpyrebot LANGUAGES CXX)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
execute_process(COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
                OUTPUT_VARIABLE PYBIND11_CMAKE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
find_package(pybind11 CONFIG REQUIRED)

if(NOT PINOCCHIO_PREFIX)
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import sys; print(sys.prefix)"
                  OUTPUT_VARIABLE PINOCCHIO_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
find_path(PINOCCHIO_INCLUDE_DIR pinocchio/multibody/model.hpp
          PATHS "${PINOCCHIO_PREFIX}/include" REQUIRED NO_DEFAULT_PATH)
find_path(EIGEN_INCLUDE_DIR Eigen/Dense
          PATHS "${PINOCCHIO_PREFIX}/include/eigen3" /usr/include/eigen3 REQUIRED)
find_library(PINOCCHIO_DEFAULT_LIB pinocchio_default
             PATHS "${PINOCCHIO_PREFIX}/lib" REQUIRED NO_DEFAULT_PATH)
find_library(PINOCCHIO_PARSERS_LIB pinocchio_parsers
             PATHS "${PINOCCHIO_PREFIX}/lib" REQUIRED NO_DEFAULT_PATH)

pybind11_add_module(_native src/bindings.cpp src/control.cpp src/robot_model.cpp)
target_include_directories(_native PRIVATE include "${PINOCCHIO_INCLUDE_DIR}" "${EIGEN_INCLUDE_DIR}")
target_compile_definitions(_native PRIVATE PINOCCHIO_WITH_URDFDOM)
target_link_libraries(_native PRIVATE "${PINOCCHIO_DEFAULT_LIB}" "${PINOCCHIO_PARSERS_LIB}")
set_target_properties(_native PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rebot_rt"
    BUILD_RPATH "${PINOCCHIO_PREFIX}/lib"
    INSTALL_RPATH "${PINOCCHIO_PREFIX}/lib")

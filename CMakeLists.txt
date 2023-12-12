cmake_minimum_required(VERSION 3.13.0)
project(ilqgames_planning
        LANGUAGES C CXX
        VERSION 0.0.1)

message("Project Version: " ${${PROJECT_NAME}_VERSION})

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CPP_VERSION 17 CACHE STRING "C++ version to use.")
set(CMAKE_CXX_STANDARD ${CPP_VERSION})

# Set the build type. Default to Debug mode.
if(NOT CMAKE_BUILD_TYPE)
  message("Defaulting to building targets in Debug mode.")
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Build options.
option(BUILD_TESTS "Build tests" ON)


# === BUILD LIBRARY === #
# Collect source files and build the ilqgames object.
file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME}/*.cpp)

# Add libraries within this package to the target module "ilqgames_planning"
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})

# Add headers within this project to the target module "ilqgames_planning"
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)


# === BUILD EXECUTABLES === #
# Add executables within this package to the target module "ilqgames_planning_main"
add_executable(${PROJECT_NAME}_main src/main.cpp)

# Add headers within this project to the target module "ilqgames_planning_main"
target_include_directories(${PROJECT_NAME}_main PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Link ${PROJECT_NAME} library to ${PROJECT_NAME}_main
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})
# set_property(TARGET ${PROJECT_NAME}_main PROPERTY CXX_STANDARD ${CPP_VERSION})


# === EXTERNAL DEPENDENCIES === #
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
                      
# rdyn_core
find_package(rdyn_core REQUIRED)
include_directories(SYSTEM ${RDYN_CORE_INCLUDE_DIR})

# ilqgames
find_package(ilqgames REQUIRED)
set_target_properties(ilqgames PROPERTIES
                      INTERFACE_COMPILE_DEFINITIONS "ILQGAMES_LOG_DIR=\"${CMAKE_SOURCE_DIR}/logs\"")
get_target_property(DEFS ilqgames INTERFACE_COMPILE_DEFINITIONS)
message("\nILQGAMES logging directory: " ${DEFS} "\n")

target_link_libraries(${PROJECT_NAME} ${ilqgames_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${ilqgames_INCLUDE_DIRS})


# === BUILD TESTS === #
if (BUILD_TESTS)
  message("Build tests is enabled.")
  add_subdirectory(test)
endif (BUILD_TESTS)
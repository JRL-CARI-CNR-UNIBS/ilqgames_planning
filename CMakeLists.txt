cmake_minimum_required(VERSION 3.13.0)
project(ilqgames_planning C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS 0)

#LIBRARY
# Collect source files and build the ilqgames object.
file(GLOB_RECURSE ${PROJECT_NAME}_srcs ${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME}/*.cpp)

# Add libraries within this package to the target module "ilqgames_planning"
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})

# Add headers within this project to the target module "ilqgames_planning"
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)


#EXECUTABLE
# Add executables within this package to the target module "ilqgames_planning_main"
add_executable(${PROJECT_NAME}_main src/main.cpp)

# Add headers within this project to the target module "ilqgames_planning_main"
target_include_directories(${PROJECT_NAME}_main PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Link ${PROJECT_NAME} library to ${PROJECT_NAME}_main
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})



# === EXTERNAL DEPENDENCIES === #
find_package(Eigen3 REQUIRED)

# Link EXTERNAL library libilqgames.a (static library in /usr/local/lib)
add_library(ilqgames STATIC IMPORTED)
set_target_properties(ilqgames PROPERTIES IMPORTED_LOCATION "${CMAKE_INSTALL_PREFIX}/lib/libilqgames.a")
target_link_libraries(${PROJECT_NAME} ilqgames)

# Include EXTERNAL headers from /usr/local/include
target_include_directories(ilqgames INTERFACE ${CMAKE_INSTALL_PREFIX}/include/ilqgames ${EIGEN3_INCLUDE_DIR})


# === COMPILER CONSTANTS === #
# Logging directory
add_definitions(-DILQGAMES_PLANNING_LOG_DIR="${CMAKE_SOURCE_DIR}/logs")

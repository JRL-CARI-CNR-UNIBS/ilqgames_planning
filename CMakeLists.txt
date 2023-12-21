cmake_minimum_required(VERSION 3.13.0)
project(ilqgames_planning
        LANGUAGES C CXX
        VERSION 0.0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CPP_VERSION 17 CACHE STRING "C++ version to use.")
set(CMAKE_CXX_STANDARD ${CPP_VERSION})

# Set the build type. Default to Release mode.
if(NOT CMAKE_BUILD_TYPE)
  message("Defaulting to building targets in Release mode.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# Build options.
option(BUILD_TESTS "Build tests" ON)


# === EXTERNAL DEPENDENCIES === #
# ilqgames
find_package(ilqgames REQUIRED)
set_target_properties(ilqgames PROPERTIES
                      INTERFACE_COMPILE_DEFINITIONS "ILQGAMES_LOG_DIR=\"${CMAKE_SOURCE_DIR}/logs\"")
get_target_property(DEFS ilqgames INTERFACE_COMPILE_DEFINITIONS)
message("ILQGAMES logging directory: " ${DEFS})
                      
# rdyn_core
find_package(rdyn_core REQUIRED)


# === BUILD LIBRARY === #
# Collect source files and build the ilqgames object.
file(GLOB_RECURSE ${PROJECT_NAME}_srcs src/${PROJECT_NAME}/*.cpp)

# Add libraries within this package to the target module "ilqgames_planning"
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_srcs})

# Link ilqgames to the target module "ilqgames_planning"
target_link_libraries(${PROJECT_NAME} ilqgames)

# Add headers within this project and ilqgames to the target module "ilqgames_planning"
# target_include_directories(${PROJECT_NAME} PUBLIC include ${ilqgames_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ilqgames_INCLUDE_DIRS}
  ${rdyn_core_INCLUDE_DIRS}
)


# === BUILD EXECUTABLES === #
# Add executables within this package to the target module "ilqgames_planning_main"
add_executable(${PROJECT_NAME}_main src/main.cpp)

# Add headers within this project to the target module "ilqgames_planning_main"
# target_include_directories(${PROJECT_NAME}_main PUBLIC include)
target_include_directories(${PROJECT_NAME}_main PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ilqgames_INCLUDE_DIRS}
)


# Link ${PROJECT_NAME} library to ${PROJECT_NAME}_main
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})


# === BUILD TESTS === #
if (BUILD_TESTS)
  message("Build tests is enabled.")
  add_subdirectory(test)
endif (BUILD_TESTS)


# === INSTALLATION === #
# Install the library and export the library target.
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the header files.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
)

set(EXPORT_CONFIGDIR "share/${PROJECT_NAME}/cmake")

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${EXPORT_CONFIGDIR}
)


# === PACKAGING === #
include(CMakePackageConfigHelpers)

set(INCLUDE_DIR include)
set(CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake/config")

configure_package_config_file(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in 
  ${CONFIG_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${EXPORT_CONFIGDIR}
  PATH_VARS INCLUDE_DIR
)

write_basic_package_version_file(${CONFIG_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CONFIG_DIR}/${PROJECT_NAME}Config.cmake
  ${CONFIG_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${EXPORT_CONFIGDIR}
)

# Create debian package (optional) using CPack
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "MikFerrari") #required
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/JRL-CARI-CNR-UNIBS/ilqgames_planning.git")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ilqgames (>= 0.0.1)") # no dependecies from other deb packages
SET(CPACK_OUTPUT_FILE_PREFIX packages) # output directory for deb package
INCLUDE(CPack)
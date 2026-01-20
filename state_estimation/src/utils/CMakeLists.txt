cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################
# DEPENDENCIES #
################

# find dependencies
find_package(Eigen3 REQUIRED)


#############
# LIBRARIES #
#############

add_library(utils INTERFACE)
target_include_directories(utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(utils INTERFACE Eigen3::Eigen)

# Install headers (preserving utils/ subfolder)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install and export the INTERFACE target
install(
  TARGETS utils
  EXPORT export_${PROJECT_NAME}
)


##############################################################################
#                                    ROS2                                    #
##############################################################################

if(DEFINED ENV{ROS_VERSION})

  # find dependencies
  find_package(ament_cmake REQUIRED)

  # Tell ament this is the exported target (modern & recommended way)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_include_directories(include/${PROJECT_NAME})


  ament_package()
endif()
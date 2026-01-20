cmake_minimum_required(VERSION 3.8)
project(
    kalman_filter
    VERSION 0.1.0
    DESCRIPTION "Kalman Filter estimator"
    LANGUAGES CXX
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Set default C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED True)
endif()

# Set default C++23
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()


################
# DEPENDENCIES #
################

# find dependencies
find_package(Eigen3 REQUIRED)

find_package(utils REQUIRED)


#############
# LIBRARIES #
#############

# linear_kalman_filter
add_library(linear_kalman_filter STATIC src/linear_kalman_filter.cpp)
target_include_directories(linear_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(linear_kalman_filter PUBLIC Eigen3::Eigen utils::utils)
set_target_properties(linear_kalman_filter PROPERTIES POSITION_INDEPENDENT_CODE ON)


# extended_kalman_filter
add_library(extended_kalman_filter STATIC src/extended_kalman_filter.cpp)
target_include_directories(extended_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(extended_kalman_filter PUBLIC Eigen3::Eigen utils::utils)
set_target_properties(extended_kalman_filter PROPERTIES POSITION_INDEPENDENT_CODE ON)


# unscented_kalman_filter
add_library(unscented_kalman_filter STATIC src/unscented_kalman_filter.cpp)
target_include_directories(unscented_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(unscented_kalman_filter PUBLIC Eigen3::Eigen utils::utils)
set_target_properties(unscented_kalman_filter PROPERTIES POSITION_INDEPENDENT_CODE ON)


###############
# EXECUTABLES #
###############

# main
# add_executable(main src/main.cpp)
# target_include_directories(src/xmain PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(main PRIVATE linear_kalman_filter)


#########
# TESTS #
#########

# # test_linear_kalman_filter
# add_executable(test_linear_kalman_filter tests/test_linear_kalman_filter.cpp)
# target_link_libraries(test_linear_kalman_filter PUBLIC linear_kalman_filter gtest gtest_main)


# # Enable testing with CTest
# enable_testing()



##############################################################################
#                                    ROS2                                    #
##############################################################################

if(DEFINED ENV{ROS_VERSION})

  ################
  # DEPENDENCIES #
  ################
  find_package(ament_cmake REQUIRED)

  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)


  #############
  # C++ NODES #
  #############

  # linear_kalman_filter_node
  add_executable(linear_kalman_filter_node src/linear_kalman_filter_node.cpp)
  target_include_directories(linear_kalman_filter_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(linear_kalman_filter_node linear_kalman_filter)
  ament_target_dependencies(linear_kalman_filter_node
    rclcpp
    tf2
    tf2_ros
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
  )
  target_compile_features(linear_kalman_filter_node PUBLIC c_std_11 cxx_std_23)


  ##############
  # INSTALLING #
  ##############

  # installing executables
  install(
    TARGETS
      linear_kalman_filter_node
    DESTINATION
    lib/${PROJECT_NAME}
	)

  # Installing directories
  install(
		DIRECTORY
			config
			launch
		DESTINATION
			share/${PROJECT_NAME}
  )


  ###########
  # TESTING #
  ###########
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()
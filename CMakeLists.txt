cmake_minimum_required(VERSION 3.22)
project(depth_port_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# ROS Dependencies =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sonia_common_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# Other Dependencies =======================================================

find_package(Boost COMPONENTS log REQUIRED)

# ==========================================================================

include_directories(include)

file(GLOB SOURCES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -Wpedantic -Werror)

set(DEPS rclcpp Boost sonia_common_cpp std_msgs std_srvs)

ament_target_dependencies(${PROJECT_NAME} ${DEPS})

if(BUILD_TESTING)
  list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
  add_library(
    ${PROJECT_NAME}_lib SHARED
    ${SOURCES}
  )
  target_include_directories(
    ${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}_lib>
    $<INSTALL_INTERFACE:include>
  )
  install(
    TARGETS ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME}_lib)
  ament_target_dependencies(${PROJECT_NAME}_lib ${DEPS})# linking the dependencies to executables

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  file(GLOB TEST_SOURCES test/*.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_SOURCES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${DEPS}
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)# permet de relier la librairie qui est en fait le ArmProvider.cpp sans le main (pour NE PAS avoir multiples main, le main du fichier test et le main des fichiers src)

  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(gui_webserver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/launch
    ${CMAKE_CURRENT_SOURCE_DIR}/shellscripts
    ${CMAKE_CURRENT_SOURCE_DIR}/webpages
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

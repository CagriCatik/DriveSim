cmake_minimum_required(VERSION 3.8)
project(autocar_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install worlds and models into the package share directory.
# GZ_SIM_RESOURCE_PATH is set at launch time to point here.
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(turtlebot3_trajectory_controller)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(controller src/trajectory_controller.cpp)
target_include_directories(controller PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
        controller
        "rclcpp"
        "geometry_msgs"
        "nav_msgs"
)

add_executable(car_controller src/car_controller.cpp)
target_include_directories(car_controller PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(car_controller PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
        car_controller
        "rclcpp"
        "geometry_msgs"
        "nav_msgs"
)

add_executable(fmt_planner src/fmt_planner.cpp)
target_include_directories(fmt_planner PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
        fmt_planner
        "rclcpp"
        "geometry_msgs"
        "nav_msgs"
)

add_executable(template src/template.cpp)
target_include_directories(template PUBLIC
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(template PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
        template
        "rclcpp"
)

install(TARGETS controller  template DESTINATION lib/${PROJECT_NAME})


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()

cmake_minimum_required(VERSION 3.16)
project(slam2d CXX)

# SLAM 2D cho xe cân bằng (SLAM-DESIGN.md). C++17, không ROS.
#   Pi:     pi/scripts/build_slam.sh
#   Laptop: cmake -S pi/slam -B build && cmake --build build -j && ctest --test-dir build

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()
# -O2: đủ nhanh, ít RAM khi biên dịch trên Pi 4 GB
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
add_compile_options(-Wall -Wno-sign-compare)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(slam2d STATIC
    src/config.cpp src/protocol.cpp src/wheel_odometry.cpp src/ekf2d.cpp src/frontend.cpp src/matcher.cpp
    src/pose_graph.cpp src/traversed_grid.cpp src/planner.cpp src/controller.cpp src/map_io.cpp src/system.cpp
    src/obstacle_grid.cpp)
target_include_directories(slam2d PUBLIC include)
target_include_directories(slam2d SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
# yaml-cpp: tên target khác nhau giữa các bản (yaml-cpp / yaml-cpp::yaml-cpp)
if (TARGET yaml-cpp::yaml-cpp)
  set(YAML_LIB yaml-cpp::yaml-cpp)
else ()
  set(YAML_LIB yaml-cpp)
endif ()
target_link_libraries(slam2d PUBLIC Eigen3::Eigen ${OpenCV_LIBS} Ceres::ceres ${YAML_LIB} Threads::Threads)

add_executable(slam_node apps/slam_node.cpp)
target_link_libraries(slam_node PRIVATE slam2d)

add_executable(slam_sim apps/slam_sim.cpp apps/sim_world.cpp)
target_link_libraries(slam_sim PRIVATE slam2d)

option(SLAM_TESTS "Build GoogleTest" ON)
find_package(GTest QUIET)
if (SLAM_TESTS AND GTest_FOUND)
  enable_testing()
  add_executable(slam_tests test/test_geometry.cpp test/test_ekf.cpp test/test_grid_planner.cpp
      test/test_pose_graph.cpp test/test_matcher.cpp test/test_system.cpp test/test_controller.cpp test/test_obstacle.cpp
      apps/sim_world.cpp)
  target_include_directories(slam_tests PRIVATE apps)
  target_link_libraries(slam_tests PRIVATE slam2d GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(slam_tests DISCOVERY_TIMEOUT 60)
elseif (SLAM_TESTS)
  message(STATUS "Không có GoogleTest (libgtest-dev): bỏ qua test")
endif ()

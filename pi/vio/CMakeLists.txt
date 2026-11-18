cmake_minimum_required(VERSION 3.16)
project(ovw_node C CXX)

# Mã nguồn OpenVINS (clone bởi pi/scripts/build_vio.sh)
set(OPENVINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/open_vins" CACHE PATH "OpenVINS source")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d video calib3d highgui)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()
# Không -g: tiết kiệm RAM khi biên dịch trên Pi 4GB. -O2 đủ nhanh, ít RAM hơn -O3.
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable)
add_definitions(-DENABLE_ARUCO_TAGS=0)

# --- Thư viện OpenVINS (ov_core + ov_init + ov_msckf), không ROS ---
file(GLOB_RECURSE OV_CORE_SRC "${OPENVINS_DIR}/ov_core/src/*.cpp")
list(FILTER OV_CORE_SRC EXCLUDE REGEX ".*/test_[^/]*\\.cpp$")
list(FILTER OV_CORE_SRC EXCLUDE REGEX ".*/dummy\\.cpp$")
file(GLOB_RECURSE OV_INIT_SRC "${OPENVINS_DIR}/ov_init/src/*.cpp")
list(FILTER OV_INIT_SRC EXCLUDE REGEX ".*/test_[^/]*\\.cpp$")
list(FILTER OV_INIT_SRC EXCLUDE REGEX ".*/Simulator\\.cpp$")
list(FILTER OV_INIT_SRC EXCLUDE REGEX ".*/dummy\\.cpp$")
set(OV_MSCKF_SRC
    ${OPENVINS_DIR}/ov_msckf/src/state/State.cpp
    ${OPENVINS_DIR}/ov_msckf/src/state/StateHelper.cpp
    ${OPENVINS_DIR}/ov_msckf/src/state/Propagator.cpp
    ${OPENVINS_DIR}/ov_msckf/src/core/VioManager.cpp
    ${OPENVINS_DIR}/ov_msckf/src/core/VioManagerHelper.cpp
    ${OPENVINS_DIR}/ov_msckf/src/update/UpdaterHelper.cpp
    ${OPENVINS_DIR}/ov_msckf/src/update/UpdaterMSCKF.cpp
    ${OPENVINS_DIR}/ov_msckf/src/update/UpdaterSLAM.cpp
    ${OPENVINS_DIR}/ov_msckf/src/update/UpdaterZeroVelocity.cpp)

add_library(openvins STATIC ${OV_CORE_SRC} ${OV_INIT_SRC} ${OV_MSCKF_SRC})
target_include_directories(openvins PUBLIC
    ${OPENVINS_DIR}/ov_core/src ${OPENVINS_DIR}/ov_init/src ${OPENVINS_DIR}/ov_msckf/src)
target_link_libraries(openvins PUBLIC Eigen3::Eigen ${OpenCV_LIBS} Boost::system Boost::filesystem
    Boost::thread Boost::date_time Ceres::ceres Threads::Threads)
target_include_directories(openvins SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

add_executable(ovw_node src/ovw_node.cpp)
target_link_libraries(ovw_node PRIVATE openvins)
install(TARGETS ovw_node RUNTIME DESTINATION bin)

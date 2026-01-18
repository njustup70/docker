# --- 1. 基本設定 ---
cmake_minimum_required(VERSION 3.8)
project(hik_camera_ros2)

# 設定 C++ 標準為 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# --- 2. 尋找依賴功能包 ---
# 尋找 ament_cmake, 這是 ROS 2 的構建系統基礎
find_package(ament_cmake REQUIRED)
# 尋找所有需要的 ROS 2 功能包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# cv_bridge 依賴於 OpenCV, 所以我們也需要找到它
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)


# --- 3. 處理海康威視 SDK (第三方庫) ---
# 重要：請確認您系統中 MVS 的實際安裝路徑與這裡填寫的一致
set(MVS_INCLUDE_DIR "/opt/MVS/include")
set(MVS_LIB_DIR "/opt/MVS/lib/64")

# 檢查路徑是否存在，如果不存在則報錯提示
if(NOT EXISTS ${MVS_INCLUDE_DIR})
    message(FATAL_ERROR "MVS include directory not found at ${MVS_INCLUDE_DIR}")
endif()
if(NOT EXISTS ${MVS_LIB_DIR})
    message(FATAL_ERROR "MVS library directory not found at ${MVS_LIB_DIR}")
endif()

# 尋找 MVS 的函式庫檔案 (libMvCameraControl.so)
find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS ${MVS_LIB_DIR}
  NO_DEFAULT_PATH
)


# --- 4. 新增可執行檔 ---
add_executable(hik_camera_node
  src/main.cpp
  src/hik_camera_node.cpp
)


# --- 5. 處理依賴關係 (核心步驟) ---

# [推薦做法] 使用 ament_target_dependencies 來處理所有 ROS 2 的依賴
# 這一個指令會自動處理好 include 路徑和 link 函式庫，非常方便
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

# [推薦做法] 使用 target_include_directories 明確地為你的節點添加第三方庫的頭文件路徑
target_include_directories(hik_camera_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${MVS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# [推薦做法] 使用 target_link_libraries 明確地為你的節點連結第三方函式庫
target_link_libraries(hik_camera_node
  ${MVS_LIBRARY}
  ${OpenCV_LIBRARIES}
)


# --- 6. 安裝 ---
install(TARGETS
  hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- 7. 完成 ament 配置 ---
ament_package()
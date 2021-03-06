cmake_minimum_required(VERSION 3.5)
project(greeting)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

# コンポーネントノードの共有ライブラリをコンパイルする
add_library(greeter_component SHARED src/greeter_component.cpp)
# コンポーネントノードのヘッダーファイルのマクロを設定する
target_compile_definitions(greeter_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(greeter_component
  rclcpp
  std_msgs
  rclcpp_components
  )
# コンポーネントノードをamentのリソースインデクスに登録する
rclcpp_components_register_nodes(greeter_component "greeter_ros2::Greeter")

# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(greeter src/greeter.cpp)
# 共有ライブラリに実行ファイルをリンクする
target_link_libraries(greeter greeter_component)
# 実行ファイルのコンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(greeter rclcpp std_msgs)

# コンポーネントノードのヘッダーファイルをインストールする
install(DIRECTORY
  include/greeting
  DESTINATION include
  )

# コンポーネントノードの共有ライブラリをインストールする
install(TARGETS
  greeter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# スタンドアローンの実行ファイルをインストールする
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

# launchディレクトリをshareへインストールする
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# amentのリソースインデクスにパッケージを登録する
ament_package()

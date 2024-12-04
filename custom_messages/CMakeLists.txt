cmake_minimum_required(VERSION 3.8)
project(custom_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/DistanceMessage.msg"
"msg/FrequencyMessage.msg"
"msg/InterruptMessage.msg"
"msg/PWMMessage.msg"
"msg/SpeedMessage.msg"
"msg/StateMessage.msg"
"msg/VoltageMessage.msg"
"msg/PercentageMessage.msg"
"msg/ControllerFeedbackMessage.msg"
"msg/DisplayBrightnessMessage.msg"
"msg/DisplayClearMessage.msg"
"msg/DisplayTextMessage.msg"
"msg/DisplayTextScrollingMessage.msg"
"msg/DisplayImageFromFilepathMessage.msg"
"srv/BuzzerServiceMessage.srv"
"action/ActionRequest.action"
DEPENDENCIES builtin_interfaces
)

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

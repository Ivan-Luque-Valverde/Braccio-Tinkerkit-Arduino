cmake_minimum_required(VERSION 3.22)
project(braccio_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS 
  scripts/pick_and_place_simple.py
  scripts/pick_and_place_configurable.py
  scripts/position_tester.py
  scripts/gripper_debugger.py
  DESTINATION lib/${PROJECT_NAME}
)

cmake_minimum_required(VERSION 3.8)
project(Main_Assembly_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch meshes textures rviz config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/five_bar_ik_node.py
  scripts/slider_gui.py
  scripts/pick_and_place_planner.py
  scripts/pick_and_place_planner_v2.py
  scripts/arduino_bridge.py
  scripts/test_stepper_angles.py
  scripts/linkage_gui.py
  scripts/linkage_gui_tk.py
  scripts/suction_hardware_node.py
  scripts/servo_node.py
  scripts/pick_place_gui.py
  # MKS SERVO42C closed-loop stepper driver (replaces arduino_bridge)
  scripts/mks_servo42c_protocol.py
  scripts/mks_servo42c_bridge.py
  # Pressure sensor ball-detection node
  scripts/pressure_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

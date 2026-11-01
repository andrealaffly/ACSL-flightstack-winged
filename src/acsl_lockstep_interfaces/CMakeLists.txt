# Simulation lockstep interfaces (identical copy in the simulator repo, see package.xml).
cmake_minimum_required(VERSION 3.8)
project(acsl_lockstep_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generates the C++ (and Python) types: acsl_lockstep_interfaces::srv::Handshake / Step
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Handshake.srv"
  "srv/Step.srv"
  "srv/Finish.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
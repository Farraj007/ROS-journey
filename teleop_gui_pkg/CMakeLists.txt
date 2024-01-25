cmake_minimum_required(VERSION 3.5)
project(teleop_gui_pkg)

# Find Python
find_package(Python REQUIRED COMPONENTS interpreter)

# Install Python scripts
install(PROGRAMS
  scripts/teleop_gui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

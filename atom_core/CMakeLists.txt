cmake_minimum_required(VERSION 3.0.2)
project(atom_core)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    scripts/calibrate
    src/atom_core/patterns.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


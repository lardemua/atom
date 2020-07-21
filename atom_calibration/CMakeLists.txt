cmake_minimum_required(VERSION 3.0.2)
project(atom_calibration)

find_package(catkin REQUIRED rostest)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    scripts/create_calibration_pkg
    scripts/configure_calibration_pkg
    scripts/collect_data
    scripts/set_initial_estimate
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_rostest(test/create_calibration_pkg.test)
endif()
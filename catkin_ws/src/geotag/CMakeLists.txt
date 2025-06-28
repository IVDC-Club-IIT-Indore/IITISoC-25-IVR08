cmake_minimum_required(VERSION 3.0.2)
project(geotag)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/battery_status.py
  scripts/Geotagger_1.py
  scripts/Geotagger_2.py
  scripts/Dual_Geotagger.py
  scripts/Static_Setpoint.py
  scripts/Dynamic_Setpoint.py
  scripts/test_geotagger_mono.py
  scripts/offboard_takeoff.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  FILES_MATCHING PATTERN "*.kml"
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


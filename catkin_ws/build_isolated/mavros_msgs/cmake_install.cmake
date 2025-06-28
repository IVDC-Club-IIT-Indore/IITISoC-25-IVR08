# Install script for directory: /root/catkin_ws/src/mavros/mavros_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/catkin_ws/install_isolated")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE PROGRAM FILES "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE PROGRAM FILES "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/setup.bash;/root/catkin_ws/install_isolated/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE FILE FILES
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/setup.bash"
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/setup.sh;/root/catkin_ws/install_isolated/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE FILE FILES
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/setup.sh"
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/setup.zsh;/root/catkin_ws/install_isolated/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE FILE FILES
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/setup.zsh"
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/setup.fish;/root/catkin_ws/install_isolated/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE FILE FILES
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/setup.fish"
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/root/catkin_ws/install_isolated/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/root/catkin_ws/install_isolated" TYPE FILE FILES "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/msg" TYPE FILE FILES
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ADSBVehicle.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ActuatorControl.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Altitude.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/AttitudeTarget.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/BatteryStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/CamIMUStamp.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/CellularStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/CameraImageCaptured.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/CommandCode.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/CompanionProcessStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/OnboardComputerStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/DebugValue.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ESCInfo.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ESCInfoItem.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ESCStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ESCStatusItem.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ESCTelemetry.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ESCTelemetryItem.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/EstimatorStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ExtendedState.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/FileEntry.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/GlobalPositionTarget.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/GPSINPUT.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/GPSRAW.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/GPSRTK.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/HilActuatorControls.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/HilControls.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/HilGPS.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/HilSensor.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/HilStateQuaternion.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/HomePosition.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/LandingTarget.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/LogData.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/LogEntry.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/MagnetometerReporter.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ManualControl.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Mavlink.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/MountControl.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/NavControllerOutput.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/OpticalFlowRad.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/OverrideRCIn.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Param.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/ParamValue.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/PlayTuneV2.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/PositionTarget.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/RCIn.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/RCOut.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/RTCM.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/RadioStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/RTKBaseline.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/State.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/StatusText.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/SysStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/TerrainReport.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Thrust.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/TimesyncStatus.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Trajectory.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Tunnel.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/VFR_HUD.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/VehicleInfo.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Vibration.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/Waypoint.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/WaypointList.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/WaypointReached.msg"
    "/root/catkin_ws/src/mavros/mavros_msgs/msg/WheelOdomStamped.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/srv" TYPE FILE FILES
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandAck.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandBool.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandHome.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandInt.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandLong.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandTOL.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandTriggerControl.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandTriggerInterval.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/CommandVtolTransition.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileChecksum.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileClose.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileList.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileMakeDir.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileOpen.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileRead.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileRemove.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileRemoveDir.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileRename.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileTruncate.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/FileWrite.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestData.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestEnd.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestList.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/MountConfigure.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/MessageInterval.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/ParamGet.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/ParamPull.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/ParamPush.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/ParamSet.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/SetMavFrame.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/SetMode.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/StreamRate.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/VehicleInfoGet.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/WaypointClear.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/WaypointPull.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/WaypointPush.srv"
    "/root/catkin_ws/src/mavros/mavros_msgs/srv/WaypointSetCurrent.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/catkin_ws/devel_isolated/mavros_msgs/include/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/catkin_ws/devel_isolated/mavros_msgs/share/roseus/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/catkin_ws/devel_isolated/mavros_msgs/share/common-lisp/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/catkin_ws/devel_isolated/mavros_msgs/share/gennodejs/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/root/catkin_ws/devel_isolated/mavros_msgs/lib/python3/dist-packages/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/root/catkin_ws/devel_isolated/mavros_msgs/lib/python3/dist-packages/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/mavros_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig.cmake"
    "/root/catkin_ws/build_isolated/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs" TYPE FILE FILES "/root/catkin_ws/src/mavros/mavros_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mavros_msgs" TYPE DIRECTORY FILES "/root/catkin_ws/src/mavros/mavros_msgs/include/mavros_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/root/catkin_ws/build_isolated/mavros_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/root/catkin_ws/build_isolated/mavros_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

cmake_minimum_required(VERSION 2.8.3)
project(audio_stream_arecord_hack)


find_package(catkin REQUIRED COMPONENTS
  rospy
)



catkin_package()


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/audio_streamer_arecord.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TODO: missing install of launch files
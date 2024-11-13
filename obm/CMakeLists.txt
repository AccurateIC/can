cmake_minimum_required(VERSION 3.0.2)
project(obm)

#################################
## Catkin package requirements ##
#################################

# Catkin package requirements
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  
)

########################################
## Install Python scripts and modules ##
########################################

# Setup to install Python scripts and modules
catkin_python_setup()

#################################
## Dynamic reconfigure options ##
#################################

# Dynamic reconfigure options


#################################
## Custom action specification ##
#################################

# Declare custom ROS actions
 

# Generate action messages
generate_messages(
  DEPENDENCIES
  std_msgs
  
)

#################################
## Catkin runtime dependencies ##
#################################

# Catkin runtime dependencies
catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  can_msg
)

#############
## Install ##
#############

# Install all the script files
catkin_install_python(PROGRAMS
  src/obm_can.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# # Install all the script config files
# install(DIRECTORY cfg/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

# # Install all the launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

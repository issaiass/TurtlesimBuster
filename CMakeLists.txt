cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_buster)

# Minimal required version for catkin
# also uses the dependencies required 
# in this project
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# To store the catking packages
catkin_package()

# The include directories
include_directories(
 ${catkin_INCLUDE_DIRS}
)

# The addition of executables
# This is an example of the talker topic
add_executable(talker src/talker.cpp)
 target_link_libraries(talker
   ${catkin_LIBRARIES}
 )

# The addition of executables
# This is an example of th listener topic
 add_executable(listener src/listener.cpp)
 target_link_libraries(listener
   ${catkin_LIBRARIES}
 )

# The addition of executables
# This is the turtlesim_buster cpp file
add_executable(moveTurtle src/moveTurtle.cpp)
 target_link_libraries(moveTurtle
   ${catkin_LIBRARIES}
 )

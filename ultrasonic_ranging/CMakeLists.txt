cmake_minimum_required(VERSION 2.8.3)
project(ultrasonic_ranging)
find_package(catkin REQUIRED  COMPONENTS
    rospy pkg_name roscpp)

## Uncomment if the package has a setup.py
catkin_python_setup()

catkin_package()

include_directories(
      ${catkin_INCLUDE_DIRS}
)

add_executable(ros_ultra src/ros_ultra.py)






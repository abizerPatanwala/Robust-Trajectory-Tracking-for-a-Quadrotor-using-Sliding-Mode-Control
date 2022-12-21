# Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control
The aim of this project is to develop a **sliding mode control** for a Quadrotor to enable it to autonomously track a trajectory. A mirco aerial vehicle (MAV) named [**Crazyflie 2.0**](https://www.bitcraze.io/products/old-products/crazyflie-2-0/) is used as a platform. The simulation of the system is done in ROS Noetic and Gazebo  
# Crazyflie 2.0 Setup in Gazebo
Run the following command below on the ubuntu terminal to install the setup files of Crzatflie 2.0.
`sudo apt update
sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
rosdep update
sudo apt-get install ros-noetic-ros libgoogle-glog-dev`


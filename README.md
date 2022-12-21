# Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control
The aim of this project is to develop a **sliding mode control** for a Quadrotor to enable it to autonomously track a trajectory. A mirco aerial vehicle (MAV) named [**Crazyflie 2.0**](https://www.bitcraze.io/products/old-products/crazyflie-2-0/) is used as a platform. The simulation of the system is done in ROS Noetic and Gazebo  
# Crazyflie 2.0 Setup in Gazebo
Run the following command below on the ubuntu terminal to install the required packages to run Crazyflie 2.0 simulation in gazebo.
- `sudo apt update`
- `sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink`
- `sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox`
- `sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev`
- `rosdep update`
- `sudo apt-get install ros-noetic-ros libgoogle-glog-dev`
Then create a new ROS worspace and run the commands below in the src folder to clone the setup files of Crazyflie 2.0
- `git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git`
- `git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git`

Then run following commands in the root of your workspace to build your package.
- `rosdep install --from-paths src -i`
- `rosdep update`
- `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False`
- `catkin build`
Now source the setup file of your workspace.

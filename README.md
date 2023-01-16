# Robust-Trajectory-Tracking-for-a-Quadrotor-using-Sliding-Mode-Control
## Overview 
[^1]: This project was done as part of a graduate course, RBE502 Controls from RBE department of WPI, by myself and [dushyant6](https://github.com/dushyant6) under the guidance of [Professor Siavash Farzan](https://github.com/sfarzan). 

[^1]In this project a **sliding mode control** for a Quadrotor is designed to enable it to autonomously track a trajectory. A mirco aerial vehicle (MAV) named [**Crazyflie 2.0**](https://www.bitcraze.io/products/old-products/crazyflie-2-0/) is used as a platform. The simulation of the system is done in ROS Noetic and Gazebo. The trajectory tracked is described below. The Quadrotor is supposed to start from origin (0,0,0) and visit 5 waypoints. The velocity and acceleration at each waypoint is zero. Using these waypoints a quintic rajectory is generated and sliding mode control is used to track the trajectory. The *output* folder contains the visualization of the trajectory tracked by quadrotor against ground truth and the video of quadrotor tracking the trajectory in gazebo.
- p0 = (0, 0, 0) to p1 = (0, 0, 1) in 5 seconds
- p1 = (0, 0, 1) to p2 = (1, 0, 1) in 15 seconds
- p2 = (1, 0, 1) to p3 = (1, 1, 1) in 15 seconds
- p3 = (1, 1, 1) to p4 = (0, 1, 1) in 15 seconds
- p4 = (0, 1, 1) to p5 = (0, 0, 1) in 15 seconds

**This project is protected under copyright policy of the course and hence the code files were not uploaded**.

## Crazyflie 2.0 Setup in Gazebo
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

## Simulation
To simulate the system, first download the *project* folder in the source folder of your workspace and build the package. Source the setup file of your workspace. Then spawn the Quadrotor in gazebo by running the following command:
`roslaunch rotors_gazebo crazyflie2_without_controller.launch`
Then run the quadrotor control file to simulate the system tracking the trajectory. The output is compared against the groundtruth and saved in *trajectory.png* file in the output folder. The video of the output can be viewed by playing *rbe502_project_video.mp4* file in the output folder.

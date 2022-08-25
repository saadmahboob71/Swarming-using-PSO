# Swarming-using-PSO
In this repository, I used ROS(Robot Operating System) with gazebo simulator to implement swarming behaviour using multiple UAVs.

You can install ROS and gazebo simulator using PX4 Autopilot.
Setup instructions are given on this link "https://www.youtube.com/watch?v=OtValQdAdrU"

After setting up your gazebo enviroment, clone this repository.


Run the following commands to get it running
cd ~/Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_uav_mavros_sitl.launch
cd ~/Swarming-using-PSO
./test.py

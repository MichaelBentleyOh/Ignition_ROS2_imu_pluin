# Ignition_ROS2_imu_pluin
This repository is for getting model data in world frame and body frame(Imu).

In ros2 control, there is "imu_broadcaster" but it gives only world frame data, which means it doesn't give imu data actually.

For this reason, I make this repository for implementing imu sensor.

In official Ignition repository, contributors gives users example that generates gazebo ignition topics for transporting simulation data
and we have to convert these topics from ignition topic to ros2 topic by "ign_ros2" package executor.

Instead of this method, I decide to make independent ros2 thread that contains plugin data into plugin so that "ign_ros2" is not needed.

Detail description will be presented below.

TBW soon

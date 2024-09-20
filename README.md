# Thesis "Integration of GNSS and LiDAR to perform the georeferenced measurement of outdoor volumes using ATLASCAR2"

# Content

- [Description](#description)
- [Installation](#installation)
- [Get a point cloud from Simulation Enviroment](#simulation)
- [Get a point cloud from Real World Enviroment](#real)
- [PCM Application](#app)

<a name="description"></a>
# Description
This thesis presents a solution for automating the georeferenced measurement of outdoor stockpile volumes using GNSS and LiDAR technologies mounted on ATLASCAR2, an autonomous ground vehicle. The system leverages high-resolution LiDAR scans for surface data and GNSS for accurate positioning to compute volumes of complex shapes like stockpiles.

The project includes:

* A ROS-based software architecture for data collection, processing, and visualization;
* An application for point cloud filtering and for accurate volume and area estimation;
* Extensive testing in both simulated (Gazebo) and real-world environments.

This work offers a robust and efficient alternative to traditional methods for volume measurement, suitable for applications in industries such as mining, logistics, and construction.

<a name="installation"></a>
# Installation

* [ROS Noetic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu))
* [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
* [LIDAR, Sick LMS151](http://wiki.ros.org/LMS1xx)
* [Novatel GPS + IMU](https://github.com/swri-robotics/novatel_gps_driver)
* [ackermann controller](http://wiki.ros.org/ackermann_steering_controller)
* [steer_drive_ros](https://github.com/CIR-KIT/steer_drive_ros)
* [pcl_ros](http://wiki.ros.org/pcl_ros)

<a name="simulation"></a>
# Get a point cloud from Simulation Enviroment

To start launch Gazebo and write:

``` 
roslaunch atlascar2_gazebo gazebo.launch world_name:=[object].world
```

Cange [object] by the name of the object to launch:

* stockpile;
* cube;
* prism.

After that, spawn the car and start the controller with the following command:

``` 
roslaunch atlascar2_bringup ackermann_bringup.launch
```

To accumulate laser scans and build a point cloud, write the following command:

``` 
roslaunch laser_to_pcl laser_assembler2.launch
```
and go around the object and record it using the command:
``` 
rosbag record /laser_pointcloud
```
Then, a .pcd file can be created using:
``` 
rosrun pcl_ros  <name.bag> laser_pointcloud ./<nameforfile>
```

<a name="real"></a>
# Get a point cloud from Real World Enviroment

Connect the right cables to the PC:

* Ethernet cable that is connected to the front switch;
* A white usb cable with GNSS labled connected to SPAN-IGM-A1;
* a black usb cable connected to SPAN-IGM-A1.

To configure the system, a few commands need to be sent to the sensor, Novatel SPAN-IGM-A1 via serial terminal, like cutecom: This are the commands:

``` 
FRESET
SERIALCONFIG COM1 230400 N 8 1 N ON
SERIALCONFIG COM2 230400 N 8 1 N ON
SETIMUORIENTATION 2
VEHICLEBODYROTATION 0.0 0.0 180.0 0.0 0.0 0.1
APPLYVEHICLEBODYROTATION enable
SETIMUTOANTOFFSET -0.80 -0.30 0.50 0.01 0.01 0.01
SETINSOFFSET 0.5 -1.3 0.5
ALIGNMENTMODE UNAIDED
SAVECONFIG
```

After the completed configuration, it is recommended to perform an alignment without turning off the sensor. This process should be conducted in an open and flat area, allowing the vehicle to travel in a straight line from 20 to 25 km/h around 20 seconds.

Use the following command to visualize all sensors data:

```
roslaunch atlascar2_bringup bringup.launch
```
To accumulate laser scans and build a point cloud, write the following command:

``` 
roslaunch laser_to_pcl laser_assembler.launch
```
and go around the object and record it using the command:
``` 
rosbag record /laser_pointcloud
```
Then, a .pcd file can be created using:
``` 
rosrun pcl_ros  <name.bag> laser_pointcloud ./<nameforfile>
```
<a name="app"></a>
# PCM Application

This application estimates the volume and area of a point cloud. There is a executable file in the VolumeCalculation folder.

Order  | Activity |
:---: | :---: |
1 | 'Choose file' button to choose point cloud
2 | Select .pcd file
3 | Apply filters from the sliders
4 | 'Visuaiization' button to visualize the point cloud with filters applied
5 | 'Save Visualization' button to image of the pointcloud
6 | 'Calculate Volume and Area' button and it will display the values of area and volume
7 | 'Export Results' Button to save results as text file

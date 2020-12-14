# DURABLE SIMULATION
This branch is dedicated to create a Gazebo simulation environment, with a multiple UGV's and UAV's, which can be used by anyone working in the project. The aim is to have a rough simulation of the real scenario where algorithms can be tested. 
The package created merges three other packages in the same simulation. These three packages are the [multi_jackal](http://wiki.ros.org/multi_jackal_tutorials), [cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo) and [BebopS](https://github.com/gsilano/BebopS) which support multiple JACKAL robots from clearpath and multiple Bebop drones. For the multi_jackal, a [forked package](https://github.com/JRosaa/multi_jackal.git) was created in order to facilitate installation and urdf file changes.

## INSTALLATION
This Gazebo solution was integrated in ROS Kinetic with Ubuntu 16. Compatibility with other versions are not ensured but might work, by installing the corresponding versions of each package. A complete guide (including both the necessary packages) is described for ROS Kinetic:

Create a ROS workspace by running:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace  # initialize your catkin workspace
cd ~/catkin_ws/
catkin build
```
### Dependencies
Open the terminal and run the following commands to install the dependencies required:
```
sudo apt-get install ros-kinetic-timed-roslaunch
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-interactive-markers-twist-server ros-kinetic-gazebo-ros-control ros-kinetic-hector-gazebo-plugins
sudo apt-get install ros-kinetic-joint-state-controller ros-kinetic-diff-drive-controller ros-kinetic-pointgrey-camera-driver ros-kinetic-robot-localization ros-kinetic-move-base
sudo apt-get install ros-kinetic-jackal-desktop
```

### Multi_jackal
This package can be installed by running:
```
cd ~/catkin_ws/src
git clone https://github.com/JRosaa/multi_jackal.git
cd ~/catkin_ws/
catkin build
```

After successful installation, procede to the next package:

### cpr_gazebo

This package is made by clearpath robotics and consists of gazebo environments which replicate the solar farm.

Intallation:
```
cd ~/catkin_ws/src
git clone https://github.com/clearpathrobotics/cpr_gazebo.git
cd ~/catkin_ws/
catkin build
```

After successful installation, procede to the next package:


### BebopS

To install this package simply follow the instructions provided in the [BebopS link](https://github.com/gsilano/BebopS). Scroll until you find the correct ROS version. In any case, the commands to run are:

```
cd ~/catkin_ws/src
git clone -b med18 https://github.com/gsilano/rotors_simulator.git
git clone -b med18 https://github.com/gsilano/mav_comm.git
git clone https://github.com/gsilano/BebopS.git
git clone https://github.com/AutonomyLab/bebop_autonomy.git
cd ~/catkin_ws/
rosdep install --from-paths src -i
catkin build
```

After successful installation, procede to the last package:

### DURABLE simulation
Lastly, install the DURABLE package by running:
```
cd ~/catkin_ws/src
git clone https://github.com/JRosaa/DURABLE.git
catkin build
```

Finally, do not forget to source the environment
```
cd ~/catkin_ws/
source devel/setup.bash
```

## USAGE
After the installation is finished, the package is ready to use. The Durable simulation consists only on joining launch files from the other packages in order to create a single gazebo environment. It is recomended to follow the same procedure when different environments are desired (with more or less robots of each type). An important note is that, the gazebo environment used is from clearpath gazebo package which has solar panels and then Jackals and UAV's are spawned into this environment. The default launch file has 4 Bebops and 3 Jackals, but this can be configured by changing/creating a new launch file. 

To change the number and initial pose of UAV please refer to the [multi_bebop.launch](launch/multi_bebop.launch) and to change the number and initial pose of UGV's to **_multi_jackal.launch_** in the forked multi_jackal package under the tutorial/launch folder. These changes are made by following the structure already present in the code. 

In order to integrate both packages, it was necessary to publish a static transform from world to map. This is present in the [created launch file](launch/durable_sim.launch).

For the UAV's waypoints change the files present in the [resources](resources).

Two launch files, with different environments were created. To switch between both, change the corresponding launch file in the [durable_sim.launch](launch/durable_sim.launch) which is the file to launch:

```
roslaunch durable_gazebo_simulation durable_sim.launch 
```
Additionally, you can change the mesh used for the environment in the file [solar.urdf.xacro](urdf/solar.urdf.xacro). Currently, there are two alternative meshes. These are to be used when calling the custom environment and not the native cpr.

Lastly, every robot creates its own namespaces with topics and its movement is controlled by each individual topic. For UGV use move base corresponding to the correct robot and for the UAV's follow Bebop website for the list of commands.

### Jackal Waypoint Publisher
It has been implemented a [node](scripts/jackal_waypoint_publisher.py) to create waypoints for each of the jackals. This node is launched by running:
```
roslaunch durable_gazebo_simulation jackal_waypoint.launch 
```
The file which contains the waypoints of the robots is [jackal_waypoints.txt](resources/jackal_waypoints.txt). This file must comply with the structure described in the first line
```
"Robot name and following lines have coordinates x,y,z roll,pitch,yaw and time which is the time the robot holds the position before moving to the next waypoint"
```
where there is a line with the robot name (same as the name for the jackal being spawned) and the others are coordinates in x,y,z roll,pitch,yaw. Lastly, there is a field for the time which the robot is to keep its position. 
During the waypoints it is possible to track the status of the robots via command line. 


### Adding Velodyne sensor to the Jackals
In order to add the Velodyne there are some required dependencies as well as some changes to the jackal urdf file.

Dependencies:
```
sudo apt-get install ros-kinetic-velodyne ros-kinetic-velodyne-*
```

UPDATE: The following section is already done. It was kept for better understanding of what was performed.

Now, it is necessary to add the velodyne to the urdf robot model. This is done in the previous installed package, multi jackal:
```
cd ~/catkin_ws/src
cd multi_jackal
cd multi_jackal_description
cd urdf 
```

open the file **_jackal.urdf.xacro_** with your prefered editor and copy the following lines just before the **_</robot>_** in the last line:
```  
  <xacro:include filename="$(find velodyne_description)/urdf/VLP-16.urdf.xacro"/>
  <VLP-16 parent="base_link" name="velodyne" topic="$(arg namespace)/velodyne_points" hz="10" samples="360" gpu="false" lasers="16" max_range="50">
    <origin xyz="0 0 0.2" rpy="0 0 0" />
  </VLP-16>
```

Velodyne configuration is done by editing the previous lines. If you have GPU set the GPS to "true".
In the case the Velodyne is not necessary, comment it out from the **_jackal.urdf.xacro_**

### Adding Realsense D435 sensor to the Jackals
This package requires [PAL robotics Gazebo package](https://github.com/pal-robotics/realsense_gazebo_plugin/tree/kinetic-devel). Install this package with the following commands:
```
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
cd realsense_gazebo_plugin/
git checkout kinetic-devel
cd ~/catkin_ws/
catkin build
```

UPDATE: The following section is already done. It was kept for better understanding of what was performed.

After installation follow the instructions [here](https://github.com/pal-robotics/realsense_gazebo_plugin/issues/7). 
The files copied should be added to the multi_jackal_description/urdf folder that should already be installed. Similarly to the Velodyne, you will need to edit the **_jackal.urdf.xacro_** file and add the following lines:
```  
  <xacro:include filename="$(find multi_jackal_description)/urdf/_d435.urdf.xacro" />
  <sensor_d435 parent="base_link" name="rs_d435" topics_ns="$(arg namespace)/rs_d435" >
    <origin xyz="0.4 0 0.2" rpy="0 0 0" />
  </sensor_d435>
```
These lines will add the camera to the front of each jackal and namespace each topic. Any configurations should be done in these lines.
In the case the Realsense is not necessary, comment it out from the **_jackal.urdf.xacro_**
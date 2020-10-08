# DURABLE SIMULATION
This branch is dedicated to create a Gazebo simulation environment, with a multiple UGV's and UAV's, which can be used by anyone working in the project. The aim is to have a rough simulation of the real scenario where algorithms can be tested. 
The package created merges three other packages in the same simulation. These three packages are the [multi_jackal](http://wiki.ros.org/multi_jackal_tutorials), [cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo) and [BebopS](https://github.com/gsilano/BebopS) which support multiple JACKAL robots from clearpath and multiple Bebop drones. 

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
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-interactive-markers-twist-server ros-kinetic-gazebo-ros-control ros-kinetic-hector-gazebo-plugins
sudo apt-get install ros-kinetic-joint-state-controller ros-kinetic-diff-drive-controller ros-kinetic-pointgrey-camera-driver ros-kinetic-robot-localization ros-kinetic-move-base
sudo apt-get install ros-kinetic-jackal-desktop
```

### Multi_jackal
This package can be installed by running:
```
cd ~/catkin_ws/src
git clone https://github.com/NicksSimulationsROS/multi_jackal.git
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

To change the number of UAV please refer to the [multi_bebop.launch](launch/multi_bebop.launch) and to change the number of UGV's to [multi_jackal.launch](launch/multi_jackal.launch). These changes are made by following the structure already present in the code. In order to integrate both packages, it was necessary to publish a static transform from world to map. This is present in the created launch file.

Two launch files, with different environments were created. To switch between both, change the corresponding launch file in the [durable_sim.launch](launch/durable_sim.launch) which is the file to launch:

```
roslaunch durable_gazebo_simulation durable_sim.launch 
```
Additionally, you can change the mesh used for the environment in the file [solar.urdf.xacro](urdf/solar.urdf.xacro). Currently, there are two alternative meshes. These are to be used when calling the custom environment and not the native cpr.

Lastly, every robot creates its own namespaces with topics and its movement is controlled by each individual topic. For UGV use move base corresponding to the correct robot and for the UAV's follow Bebop website for the list of commands.

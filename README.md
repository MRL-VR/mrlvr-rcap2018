# MRLVR 2019

Here is MRL VR team source code from Qazvin Islamic Azad University.

Note: This is primiry version of readme.md file and it will be complete.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
- Ubuntu 16.04.3 LTS
- ROS Kinetic Desktop Full
- QTCreator 5.1

```

### Installing

A step by step series of packages and softwares that you need to install them.

Install ROS Kinetic, Gazebo and QT Creator
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 

sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install git ros-kinetic-desktop-full ros-kinetic-hector-map-tools ros-kinetic-costmap-2d ros-kinetic-hector-marker-drawing ros-kinetic-nav-core ros-kinetic-base-local-planner ros-kinetic-navfn ros-kinetic-move-base

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

Clone & build  MRLVR 2019 project from gitlab repository:

```
git clone https://gitlab.com/mrlvr/vr2019.git

cd ~/vr2019

catkin_make

echo "source /home/[your username]/vr2019/devel" >> ~/.bashrc

source ~/.bashrc
```
Clone and & build RoboCup2018 Rescue Simulation Virtual Robot League Maps:
```
sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol libignition-math3 libsdformat5 libignition-math3-dev libignition-msgs0-dev gazebo8-plugin-base libgazebo8 libgazebo8-dev ros-kinetic-desktop  ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-image-view2 ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-message-to-tf ros-kinetic-tf2-geometry-msgs ros-kinetic-audio-common ros-kinetic-costmap-2d ros-kinetic-image-transport ros-kinetic-image-transport-plugins ros-kinetic-hector-mapping ros-kinetic-hector-geotiff Ros-kinetic-master-discovery-fkie ros-kinetic-master-sync-fkie ros-kinetic-pose-cov-ops

sudo apt-get install ros-kinetic-hector-pose-estimation ros-kinetic-hector-sensors-description ros-kinetic-controller-manager ros-kinetic-gmapping ros-kinetic-move-base ros-kinetic-hector-mapping ros-kinetic-gazebo8*

git clone https://github.com/reyhanehpahlevan/RoboCup2018RVRL_Demo.git

cd ~/RoboCup2018RVRL_Demo

./cleanup

catkin_make
```

## Running the server
To run maps and spawn robots series of commands is need to run:

- Create run-server.sh file in your home direcotory:
```
gedit run-server.sh
```
- Copy these commands in file:
```
cd /home/your username]/RoboCup2018RVRL_Demo/ 
. setup.bash
roslaunch setup world_final_indoor_1.launch
```
- Save file and close it

- Run these command to set executable attribute on file:
```
chmod +x run-server.sh
```
- Create spawn-robot.sh file in your home direcotory:
```
gedit spawn-robot.sh 
```

- Copy these commands in file:
```
cd /home/your username]/RoboCup2018RVRL_Demo/ 
. setup.bash
roslaunch robot_description spawn_multi_robots_final_indoor_1.launch
```

- Save file and close it.

- Run these command to set executable attribute on file:
```
chmod +x spawn-robot.sh
```

- Open Terminal 1:
```
./run-server.sh
```

- Open Terminal 2:
```
./spawn-robot.sh
```

## Running the MRLVR 2019 GUI
Run bellow command to start mrl_rqt_dashboard:
```
rosrun rqt_gui rqt_gui
```
note: If you can not find mrl->dashboard menu from rqt_gui, run bellow command, after repeat previous command:
```
rqt --force-discover
```


## Authors


* **Mohammad H. Shayesteh** - *Software developer* - (m.h.shayesteh@gmail.com)


* **Mohammad M. Raeisi** - *Software Developer* - (mahdrsi@gmai.com)

## License

This project is licensed under the Mechatronics Research Laboratory.


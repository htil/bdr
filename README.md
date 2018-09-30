# Brain Drone Race

## Getting Started

Clone this repository on your local machine.

'''
git clone https://github.com/htil/BDR.git
'''

### Prerequisites

1. Have a machine running Ubuntu 16.04 (only version of Linux verified working)
2. Install ROS Kinectic: http://wiki.ros.org/kinetic/Installation
3. Install bebop_autonomy: https://bebop-autonomy.readthedocs.io/en/latest/installation.html

### Installing

Change your current directory to BDR, initialize your catkin workspace, build your catkin workspace, and source the setup.bash script.

```
$ catkin init
$ catkin_make
$ source devel/setup.bash
```

### Running the Code

Open a terminal and launch the bebop_driver.

```
$ roslaunch bebop_driver bebop_node.launch
```

Open a new terminal and run the sensor node.

```
$ rosrun bebop_brain_drone_race sensor.py
```

Open a new terminal and run the controller node.

```
$ rosrun bebop_brain_drone_race controller.py
```

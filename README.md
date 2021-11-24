# fetch-delivery-system
A delivery and retrival system utilizing a Fetch Mobile Manipulator robot

## Installation
1. [Install Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
2. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
3. [Set up a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
4. Run the following commands in a Bash terminal in the catkin workspace you just created:
```
sudo apt install python3 python3-pip # install Python3 and Pip3
pip install ipython numpy # install IPython and NumPy
cd src # change to the src directory
git clone https://github.com/jamesdarrenmuir/fetch_api.git # clone the Fetch API
git clone https://github.com/GIX-C4RT/fetch-delivery-system.git # clone this repository
cd .. # change to catkin workspace directory
catkin_make # build the packages
```
5. Perform the fetch_moveit_config bug workaround from the Known Issues section

## Running
### Pick-and-Place
#### Pick
1. Navigate to the catkin workspace directory.
2. Run the following commands in a Bash terminal in the catkin workspace:
```
source devel/setup.bash # overlay workspace on terminal environment
roslaunch fetch_delivery_system pick.launch # launch the pick code
```

## Known Issues
### fetch_moveit_config Bug Workaround
The fetch_moveit_config package has a bug in one of the files. In order to fix it, please open a Bash terminal,
navigate to your catkin root directory, then running the following commands:
```
sudo cp ./src/fetch-delivery-system/workarounds/fetch_moveit_config/ompl_planning.yaml /opt/ros/melodic/share/fetch_moveit_config/config
```

### RViz and VMWare
RViz (and Gazebo, and some other software) does not seem to work well with the graphics drivers when running Ubuntu in a VMware.
See this [Gazebo Answers post](https://answers.gazebosim.org//question/13214/virtual-machine-not-launching-gazebo/)
for more info and instructions for how to fix.

TL;DR: enter `echo "export SVGA_VGPU10=0" >> ~/.profile` in a Bash terminal

If you don't want to permanently change your environment variables, 
[try launching RViz with the following option](https://github.com/ros-visualization/rviz/issues/1544#issuecomment-690338537):
`rosrun rviz rviz --opengl 210`


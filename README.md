# fetch-delivery-system
A delivery and retrival system utilizing a Fetch Mobile Manipulator robot

# Installation
1. [Install Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. [Set up a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
3. Run the following commands in a Bash terminal in the catkin workspace you just created:
```
sudo apt install python3 python3-pip # install Python3 and Pip3
pip install ipython # install IPython
cd src # change to the src directory
git clone https://github.com/jamesdarrenmuir/fetch_api.git # clone the Fetch API
git clone https://github.com/GIX-C4RT/fetch-delivery-system.git # clone this repository
cd .. # change to catkin workspace directory
catkin_make # build the packages
```

## Running Pick-and-Place Code
### Pick
Terminal 1: `roslaunch fetch_moveit_config move_group.launch`
Terminal 2: `rosrun robotics_labs segment_object`
Terminal 3: `python moveit_final.py`

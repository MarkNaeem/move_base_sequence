# Move base sequence
## Overview
This is a ROS package that uses a ROS Action server to manage sending multiple goals to the navigation stack (move base action server) on a robot in order to achieve them one after another. The package handles everything regarding the goals: recieving, storing, sending, error handling... etc. 

 
 ## Dependencies
This package was made on ROS Melodic. It is compatible with any ROS distribution that supports the following packages:

- **move_base:**   `sudo apt-get install ros-$ROS_DISTRO-move-base`
    - move_base_msgs 
- **actionlib:**   `sudo apt-get install ros-$ROS_DISTRO-actionlib`
    - actionlib_msgs 
- **gemoetry_msgs:** `sudo apt-get install ros-$ROS_DISTRO-geometry-msgs`
- **std_msgs:**     `sudo apt-get install ros-$ROS_DISTRO-std-msgs`

Note that if you're using the binaries release as mentioned in the installing section, the missing dependencies will be installed automatically.

A guide on how to get the ROS distro that's compatible with your system:<br>[Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)<br> [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)


## Installing
### ROS Distribution
The package is to be released in ROS Kinetic, Melodic, and Noetic distributions. ROS2 will be the next step.

### Build from source
To install the package,clone this repo `git clone https://github.com/MarkNaeem/move_base_sequence.git` in your catkin workspace, which is usually `~/catkin_ws`, and build the package using `catkin_make --pkg move_base_sequence`, or by just using `catkin_make` to build the whole workspace.
It is recommended to run **[rosdep](http://wiki.ros.org/rosdep)** `rosdep install move_base_sequence` before building the package to make sure all dependencies are properly installed


## Nodes, topics, services, parameters
#### Nodes: 
- ***move_base_sequence:*** the one that runs the actionlib server that handles everythig about the goals sequence handling.

#### topics:
- ***wayposes:***  PoseArray, a visulaization topic that shows the regiestered goals. It can also be sued to pass precalculated set of goals all at once by publihsing on it.
- ***path:*** Path, visualization topic that draws the path that connects goals together.
- ***corner_pose:*** Pose, the topic that is used to append new goals to the goals sequence.

#### services:
<sub>**note**:
 A robot using move base sequence can have two states:
 *paused:* paused state stops the move base server and stops the sequence server so the robot stays at its place.
 *operating:*  operating state means that the sequence server will be sending goals and waiting for move base response. In other words, the system will be fully functioning until something casues state to change to paused (e.g. a goal cancellation or abortion)
</sub> 

- ***toggle_state:*** toggles the robot state from paused to operating.
- ***set_state:*** sets the state of the robot using `True` (opetating), `False` (paused).
- ***reset:*** deletes all registered goals and cancel any sent one. The node is then set to how it started.
- ***get_state:*** returns the current state of the server, either `True` (opetating), `False` (paused).

#### Parameters
- ***abortion_bahaviour:*** determines the behaviour of the robot should the move base server face any problems that causes goal abortion. Default is `'stop'` but it can be set to `'continue'` which will make the system ignore this goal and take the next one in the sequence. It takes `'continue'` and `'stop'` as strings.
- ***is_repeating:*** determines whether the robot should be looping on the goals in an infinite loop which is the default value and set to be `True`, or it should only achieve them once, delete them, and wait for new goals, which is set by `False`.



## How to use it
#### What to do before using the package
To use the move base sequence package, all you need is to have your move base action server running (aka setup the navigation stack on your robot). [Navigation stack]('http://wiki.ros.org/navigation') has great [tutorials]('http://wiki.ros.org/navigation/Tutorials') and a detailed explanation about the whole stack and how it works.

#### Sequence server setup
The server runs thorugh `move_base_sequence` node, which is initialized in `server.py`. 

**It can be called via:**
+ launch files:
```
<node name="move_base_sequence" pkg="move_base_sequence" type="server.py">
       <param name="/move_base_sequence/is_repeating" value="True"/>
       <param name="/move_base_sequence/abortion_behaviour" value="stop"/>
    </node>
```

+ or `rosrun` command: 
```rosrun move_base_sequence server.py```


#### Adding goals to the sequence server

To add goals to the sequence, there is three ways:
+ Change the topic of `2D nav goal` tool on rviz to `/move_base_sequence/corner_pose` as follows:
1- Click on panels in the menus bar, a drop down menu will appear.
2- From the menu, check `Tools properties` if not checked.
3- A new window will show up, scroll down this menu unitl you get to `2D nav goal` and change its topic to `/move_base_sequence/corner_pose`.<br>
With this, every time you use the `2D nav goal` GUI tool on rviz, the topic published will be `/move_base_sequence/corner_pose` not the default `/move_base_simple/goal`. The following GIF shows how to do the mentioned steps.
+ Publish the targeted pose directly without rviz on the topic `/move_base_sequence/corner_pose`.
+ Publish the whole targeted set of poses on the topic `/move_base_sequence/wayposes`.

## Examples
#### Running move_base_sequence on turtlebot3

#### Running move_base_sequence on robo_farm


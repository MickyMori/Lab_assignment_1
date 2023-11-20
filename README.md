Experimental Robotics Laboratory - First Assignment
======================================
Aruco Marker Robot Navigation
======================================

Project Description
-------------------------

This project involves controlling a robot's movement based on the detection and interaction with Aruco markers in its environment. The robot navigates through a predefined sequence of markers until it reaches a designated target marker.

Create and setup a Catkin Workspace
--------------------------------

A catkin (ROS) workspace is a directory in which you can create or modify existing catkin packages. We will label our catkin workspace `catkin_ws`. To create the catkin workspace, type the following commands in the Terminal:

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
```

This will allow us to create a new folder in the home directory called `catkin_ws` by using the `mkdir` command. Then we create a source `src` folder inside the catkin workspace folder. Once done, make sure to use the command `catkin_make` inside the `catkin_ws` folder in order to init the catkin workspace. If you view your current directory contents, you should now have the `devel` and `build` folders. Inside the `devel` folder there are now several `setup.*sh` files. 

Then, we will need to source the `setup.bash` file to overlay this workspace on top of our ROS environment. In order to do this, it is necessary to go back in your home directory with the `cd` command and then type:

```bash
source ~/catkin_ws/devel/setup.bash
```

Remember to add this source command to your `.bashrc` file under the source line of the global ROS installation. This is important so as to use your code with ROS functionalities. In order to access the `.bashrc` file type the following command:

```bash
gedit ~/.bashrc
```

How to run the simulation
-------------------------

Move inside the package and clone our repository and the one containing the markers models by typing the commands:

```bash
git clone https://github.com/CarmineD8/aruco_ros.git
git clone https://github.com/MickyMori/Lab_assignment_1.git -b rotating_camera
```

At first, to utilize the marker textures, copy the `models` folder from the `aruco_ros` package into `/root/.gazebo/` directory (all new models should be put here, to let the camera work properly).

... Explanations about camera link etc ...

Then, since inside the source folder there is new content, you need to type the command `catkin_make` inside the ROS workspace folder:

```bash
cd
cd catkin_ws
catkin_make
```

Finally, run the whole project by running the launch file:

```bash
roslaunch lab_assignment world.launch
```

Flowchart
-----------------------

Logic Node (Logic_node.cpp)
* Subscribes to Aruco marker messages.
* Controls the camera in order to find a new target.
* When a target is found, aligns the base frame with the camera's orientation.
* Controls the robot's movement based on marker detection.
* Publishes velocity commands and marker IDs for searching.

Aruco Marker Publisher Node (CV_node.cpp)
* Subscribes to target and camera feed messages.
* Detects Aruco markers by processing them.
* Publishes marker information for the Logic Node to control robot movement.

![Flowchart of the robot behaviour](lab_assignment/media/Flowchart_rotating.png)

Node Graph
-----------------------

![Rqt Graph](lab_assignment/media/rosgraph_rot.png)


Simulation
-----------------------

https://github.com/MickyMori/Lab_assignment_1/assets/104144305/4ddfdac8-d989-4ddd-af9c-f3ed96d629f1

Code Documentation
-----------------------

add generated docs link...

Possible Improvements
-----------------------

* Implement a more sophisticated navigation algorithm for smoother movement between markers.
* Enhance error handling and recovery mechanisms for better resilience in marker detection scenarios.

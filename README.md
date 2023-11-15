Experimental Robotics Laboratory - First Assignment
======================================

Project Description
-------------------------

The main objective of the project is controlling a robot's movement based on the detection of Aruco markers. Aruco markers are visual markers used in robotics. The program employs a proportional controller to adjust the robot's orientation based on the marker's position. The robot sequentially tracks a predefined set of Aruco markers, aligns its center with each marker, and moves forward when the alignment is achieved. 

explain what the project is about...

talk about rosbot_ros package...

objectives of the project...

How to run the simulation
-------------------------

Move inside the package and clone our repository and a mandatory one (note that we are using ROS Noetic distro) by typing the command:

```bash
git clone https://github.com/husarion/rosbot_ros.git -b noetic
git clone https://github.com/MickyMori/Lab_assignment_1.git
```

Then, since inside the source folder there is new content, you need to type the command `catkin_make` inside the ROS workspace folder:

```bash
cd
cd <your_workspace>
catkin_make
```

Open a new tab in the terminal and run the whole project by running the launch file:

```bash
roslaunch lab_assignment world.launch
```
Software Architecture 
-----------------------

imaage ...

explain the software architecture used ...

Flow Chart 
-----------------------

image...

explanation...

Simulation
-----------------------

video of the simulation...

environment used...

behaviour explanation...

Code Documentation
-----------------------

add generated docs link...

Possible Improvements
-----------------------

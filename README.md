Experimental Robotics Laboratory - First Assignment
======================================

How to run the simulation
-------------------------

Start by installing the `xterm` library. This can be done running on terminal the following code:

```bash
sudo apt-get install konsole
```

`Note`: xterm is the standard terminal emulator in the Unix-like environment. A user can have multiple xterm sessions started on one or more displays, which provide an input/output system for the processes launched.

You also need to clone a mandatory repository by typing the following command (note that we are using ROS Noetic distro):

```bash
git clone https://github.com/husarion/rosbot_ros.git -b noetic
```

Finally, you can move inside the package and clone the repository by typing the command:

```bash
git clone https://github.com/MickyMori/Lab_assignment_1.git
```

Then, since inside the source folder there is new content, you need to type the command `catkin_make` inside the `catkin_ws` folder:

```bash
cd
cd catkin_ws
catkin_make
```

Open a new tab in the terminal and run the whole project by running the launch file:

```bash
roslaunch lab_assignment world.launch
```

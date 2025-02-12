# HM1 RL 24/25 Arsen Hudyma

Hi, this is the repository for the HM1 of the course 'Robotics Lab' and this time our focus will be on loading a robot manipulator in rviz2 and in gazebo worlds, but also on the implementation of the ros2_control part so that we can send position commands to our robot!
To start, you'll need this repository on your computer, so get it by:
```bash
git clone https://github.com/arsen-hdm/RL_HM1.git
```
Apart of the repository, give a look at my presentation of this HW so that it can be more clear and for better understanding of the things I've done.
Consider that it's a powerpoint and in some parts it may contain videos, so remember to look at it ;)

Then, once you're in the dockek container, firstly:
```bash
colcon build
. install/setup.bash
```

### Simulation in rviz2
Now, you can run the first and simpler simulation, the one in the rviz2 environment with also the collision shapes of the robot (you must activate it in rviz2 via the topic visualization).
To do so the command is:
```bash
ros2 launch arm_description display.launch.py
```

### Simulation in gazebo
If instead you want to run the simulation in gazebo, with the camera sensor being mounted and also the ros2_control integration you must run this other command:
```bash
ros2 launch arm_gazebo arm_world.launch.py
```

### Sending position commands
For last, if you want also to send some position commands to the robot so that it moves as desired, you must run the previous command but also:
```bash
ros2 run arm_control control_node
```
Remember that if you want to change the desired positions of the joints you need to do simple modifications in the node src code of the arm_control package. The part of code is specified in the presentation with also the path to it.
Thanks for the attention, you can find also other homeworks in my personal repositories!

# TurtlesimBuster

<details open>
<summary> <b>Brief Review<b></summary>
Note that the preffered language of this tutorial is using 
<img src="https://img.shields.io/badge/-C++-0390fc?style=for-the-badge&logo=c&logoColor=white&labelColor=4B8BBE" />

<p align="center">
<img src = "https://github.com/issaiass/TurtlesimBuster/blob/master/imgs/turtlesim_buster.PNG?raw=true" width="40%"/>
</p>

<p align = "center">

A turtlesim program that bust new spawned turtles in the field. 

This programs will help you to understand the basic ROS concepts like nodes, publishers, subscribers and services.  

Turtlesim is a basic simulator where you can test the main concepts of ros like messages, topics, publisher, subscribers, services, actions and so on.  Do not subestimate the simulator because a code that will run here could run on your robot (i.e.  turtlebot) will minimal modifications, off course this simulator has its own limitations. 

This applications function as follows. 
- First, we spawn a 'leader' turtle. The box frame is (0,0) at the lower left and near (12,12) at the top right.
- The spawned turtle will land on a random location of this field.
- Next, the current turtle in the turtlesim simulator (named by default as turtle1) will chase the 'leader' turtle
- For chasing we use two PID controller
  - PID_distance will make control of the distance to achive
  - PID_angle will make control of the angle to achieve
- You can experiment feedback control by modifying those PID values
- When the turtle reaches a minimal 'gotcha' distance it will kill the current 'leader' turtle and will spawn a new 'leader' turtle
- The process repeats indefinitely.

**If you want to run the project directly just go to the section <Using TurtlesimBuster Package>, but you at least have to complete these prerequisites:**
- **Installed ROS**
- **Know how to create a workspace**
- **Have a minimal knowledge of ROS topics, messages, services, launch files**

Quick reference that could help you in your application are [this for implementing the PID controller if you do not remember how](https://gist.github.com/bradley219/5373998) and [this one from the IRIG group of UPC Spain](https://sir.upc.edu/projects/rostutorials/5-client-server_tutorial/index.html).




</details>

<details close>
<summary> <b>The ROS Basics<b></summary>

In this section we will cover:
- Installation of Ubuntu
- Installation of ROS
- The catkin workspace
- Package creation
- Launching Turtlesim
- Making Launch files (the basics)



A. Ubuntu and ROS Installation
1. Install Ubuntu 20.04 or other [using this guide](https://phoenixnap.com/kb/install-ubuntu-20-04).
2. Install ROS Noetic Ninjemys or your favorite ROS distribution by [following the ROS installation wiki](http://wiki.ros.org/noetic/Installation).
3. Insert this line at the end of the `~/.bashrc` file for sourcing ROS, save and exit.
    source /opt/ros/noetic/setup.sh
4. Just this time, source ROS on your terminal by typing `source ~/.bashrc` and press enter.

B. Creating a workspace and a package

5. Create a ROS ros workspace and compile an empty package:

    `cd ~`   
    `mkdir -p catkin_ws/src`
    `cd catkin_ws`
    `catkin_make`
6. Open the .bashrc with nano

    `nano ~/.bashrc`
7. Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
    `source ~/catkin_ws/devel/setup.bash`
8. Create a package and its dependencies (inside `~/catkin_ws/src` folder)

`catkin_create_pkg turtlesim_buster roscpp std_msgs`

9. Ensure again you can compile the package doing `catkin_make` in the root folder `catkin_ws`.
    
    `cd ~/catkin_ws`
    `catkin_make`

C. Inspecting package.xml and CMakeLists.txt from your package folder

10. Inspect the file in `~/catkin_ws/src/turtlesim_buster/package.xml` and look at the lines 51-57, those are build dependencies and excecution dependencies created by our previous command to run the package.

11. Inspect also the CMakeLists.txt inside `~/catkin_ws/src/turtlesm_buster/CMakeLists.txt` and inspect the lines:
  - 1-2: this is the cmake minimun version required by ROS and how is named the package to compile
  - 10-12:  whick packages catkin will require to run this package
  - 104: some other packages that we will need to run this package, by default right now not necessary.
  - 117-120: here will go the package includes first and the catkin include, by now we are ok.

D. Launching Turtlesim and creating a Launch File

We can then run turtlesim.

12. Open a 2nd Terminal and run:

    `roscore`

ROS needs `roscore` to run before all services, nodes and everything are trying to connect there.

13. Try to launch turtlesim by typing the next command:

    `rosrun turtlesim turtlesim_node`

`rosrun` as you see is to run a node.  Think that a node is a program (basically).  And the default command to run a node is `rosrun <PACKAGE_NAME> <EXECUTABLE>`.

13. Open a 3rt terminal window and run this command

    `rosrun turtlesim turtle_teleop_key`

14.  Finally, in this terminal window try to manipulate the turtle with your keyboard arrows.

E. Creating Launch Files

As you see this is tedious to open terminals, launch the roscore, launch turtlesim and other nodes.  Let's create a launch file:

15.  Create a new folder named `launch` at the same level of your package, that is at `~/catkin_ws/src/turtlesim_buster/`.

    `mkdir launch`

16.  Now name a launch file as you want. For simplicity I always make it similarly to the package name, at least the first part.

`touch turtlesim_buster_launchexample.launch`
`nano turtlesim_buster_launchexample.launch`

17.  Now write the following lines inside the launch file, save it and exit.

    <launch>
        <node name="this_runs_turtlesim>" pkg="turtlesim" type="turtlesim_node" output="screen" />
        <node name="this_runs_teleop_key>" pkg="turtlesim" type="turtle_teleop_key" output="screen" />  
    </launch>

18. Open a terminal, run the next command and click on this window to move the turtle again.

    `roslaunch turtlesim_buster turtlesim_buster_launchexample.launch`

Great no? Now with a single command we run `roscore`, launch `turtlesim` and also `turtle_teleop_key` all by the package name `turtlesim_buster`.

In resume the command for launching files is:

 `roslaunch <PACKAGE_NAME> <LAUNCH_FILE>`

19. Good, now you now some of the mininimal basics of ROS.  Let's move on other situations we can explore using turtlesim.

</details>

<details close>
<summary> <b>RQT Graph, ROS Topics, Publisher, Subscribers, Services and Messages<b></summary>
A. RQT Graph

A graph display the connection between nodes and how they communicate each other.

<p align="center">
<img src = "https://github.com/issaiass/TurtlesimBuster/blob/master/imgs/rqt_graph.PNG?raw=true" width="70%"/>
</p>

1.  For running the graph, open a terminal and write the command `rqt_graph`.

In resume a node is a program like the `turtlesim_node`, the node can publish o subscribe to `topics`

As the image above lists, the `turtlesim_node` and `teleop_key` are nodes and their relationship is a topic.

B. ROS Topics

Topics, in a brief can be publishers or subscribers, these topics could give us information relative to control or just information of the node. 

2. To list the topics use the command `rostopic list`.

Here is the topic `turtle1/cmd_vel`.

3. Explore a topic by running the command `rostopic`, there are a list of functions, but let's explore the topic for turtlesim only, the velocity command by `rostopic echo /turtle1/cmd_vel`.

4. If you move the turtle by the `turtlesim_teleop_key` node you will see the current output like:

    `rostopic echo /turtle1/cmd_vel`
    `linear:`
    `x: 0.0`
    `y: 0.0`
    `z: 0.0`
    `angular:`
    `x: 0.0`
    `y: 0.0`
    `z: 2.0`

In resume to move the turtle we can give a linear velocity by `x` or an angular velocity by `z`.  The linear velocity will push the robot back and forth (if negative) and the angular velocity will make the robot turn counter clockwise or clockwise (if negative).

5.  Explore the topic `/turtle1/pose` with `rostopic echo /turtle1/pose` and move the robot.  Do the same with the `/turtle1/color_sensor` topic.

C. Publishers and Subscribers

The topics could be published or subscribed, here the `/turtle1/cmd_vel` is both.

For the `turtle_teleop_key` is publishing the command and the `turtlesim_node` is subscribed to this topic.

Depending of how is constructed the application there are several topics that we could be explored.

6.  Move the turtle by publishing the topic cmd_vel with the command:

    `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:`
    `x: 1.0`
    `y: 0.0`
    `z: 0.0`
    `angular:`
    `x: 0.0`
    `y: 0.0`
    `z: 0.0"`

7.  The robot after running foward we could get back by moving to the previous position by the command

    `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:`
    `x: -1.0`
    `y: 0.0`
    `z: 0.0`
    `angular:`
    `x: 0.0`
    `y: 0.0`
    `z: 0.0"`

8.  Repeat the same but for the angular velocity and make the robot turn by publishing to the same topic but the angular velocity.

E. Messages.

Messages are the structur of how are constructed the information to send to the nodes, each topic has an specific message.

9. For exploring more about the messages in the topic we could use the next command:

    `rostopic info /turtle1/cmd_vel`

10. The output will be something like:

    `Type: geometry_msgs/Twist`
    ` `
    `Publishers:`
    ` /this_runs_teleop_key (http://robot:46149)`
    `Subscribers:`
    ` /this_runs_turtlesim (http://robot:35053)`

11.  Here `geometry_msgs/Twist` is a type of message.  Again, think that messages like structure of data in language programming like C/C++.

12. Let's explore the messages by looking at the folder:

    `roscd geometry_msgs`
    `cd msg`
    `ls`

13.  As you see there is a list of default messages that we can use, and as expected, there is the `Twist.msg`

14.  Open the twist message in nano by `nano Twist.msg`.  You see that the message is composed by the type `Vector3` and named `linear` and `angular` respectively.

15.  Exit nano and explore then the `Vector3.msg`.  Here we can see that the message is composed by:

    `float64 x`
    `float64 y`
    `float64 z`

16.  Now, as everything is comming to make sense, let's explore other utility commands that ROS has for messages:

    `rosmsg list | grep turtle`

17.  Here we can also see a usefull message named `turtlesim/Pose`:

    `float32 x`
    `float32 y`
    `float32 theta`
    `float32 linear_velocity`
    `float32 angular_velocity`

18.  If you remember, there was a topic named `turtle1/pose`.  If you review that topic, you will see that there is a pose message as the previous commented lines.

19.  Move the robot by the `/turtle1/cmd_vel` and in a separate window view the topic `/turtle1/pose`, you will see when moved, the output format of the data concords with the pose message.

E. Services and Actions

Services and Actions are other kind of 'functions' that we can use in ROS.  The main difference of services and actions are that with services we have to wait until it finishes and with actions we can cancel at any time.  

Actions are more complicated to rule so we will state by the moment with ROS services.

20. View the available commnds by `rosservice` and press enter.

21. List all services by `rosservice list`

22. Lets explore the `/spawn` and `/kill` services.

23. Create a turtle named `new_turtle` by typing the command:

    `rosservice call /spawn "x: 3.0 y:3.0 theta: 0.0 name:'new_turtle'"`

24. If everything goes well this will spawn a new turtle at the position (3,3).

25. Now kill the turtle by the service:

    `rosservice call /kill "name:'new_turtle'"`

26. You can continue exploring new horizons of ROS, but this will cover th basics and neccesary knowledge if you are a newbie for the application that we developed here.

</details>

<details close>
<summary> <b>ROS Dependencies in CMakeLists.txt and package.xml<b></summary>

1. Explore the dependencies of the CMakeLists.txt of this repository.

2. Here only concentrate on the `MoveTurtle` that corresponds to the `MoveTurtle.cpp` in the package.

3. As you see we need to specify for the package source `MoveTurtle.cpp` the name and its dependencies for compilation for cmake.

4. That's all, explore the packages.xml of this repo to see more information about the build dependencies.

</details>

<details open>
<summary> <b>Using TurtlesimBuster Package<b></summary>

- Create a ROS ros workspace and compile an empty package:
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
~~~
- Open the `.bashrc` with nano:
~~~
    nano ~/.bashrc
~~~    
- Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
~~~
source ~/catkin_ws/devel/setup.bash
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/TurtlesimBuster
    mv TurtlesimBuster turtlesim_buster
    rm -rf README.md
    rm -rf imgs
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make`.
- Finally launch the application by:
~~~
    roslaunch turtlesim_buster turtlesim_buster.launch
~~~
- You must see that `roscore` executes, `turtlesim_node` executes and also a turtle is chasing a static turtle.

</details>

<details open>
<summary> <b>Results<b></summary>

You could see the results on this youtube video.  

<p align="center">

[<img src= "https://img.youtube.com/vi/JLOHCpsOIlA/0.jpg" />](https://youtu.be/JLOHCpsOIlA)

</p>

The video only shows the application running, not the explanation of the code.

Below a simple image of the application:

<p align = "center">
<img src = "https://github.com/issaiass/TurtlesimBuster/blob/master/imgs/chasing_turtle.PNG?raw=true" width="50%"/>  
</p>

</details>

<details open>
<summary> <b>Video Explanation<b></summary>

I will try my best for making an explanatory video of the application as in this view.

<p align="center">

[<img src= "https://img.youtube.com/vi/PMTdrLLh_gw/0.jpg" />](https://youtu.be/PMTdrLLh_gw)

</p>

</details>

<details open>
<summary> <b>Issues<b></summary>

Currently are no issues present.

</details>

<details open>
<summary> <b>Contributiong<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>

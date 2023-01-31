# robutler_psr_2022

This works was made in the ambit of the corse robotic systems programing. This will make use of ROS, and two well known robots, for the movement it will be used the turtelbot waffle pi. For manipulating the environment it wil be used the BCN3D MOVEO, a open sorce and 3D printable manipulator with 5 degrees of freedom.

## Project description

The objective is to develop a robotic system that acts like a butler. To accomplish this the robot needs to be capable of preform a set o functionalities, such as>

1. Create a map of the enviroment in witch is inserted.
2. Move automatically tro the environment without colliding with it.
3. Have the capability to perform basic functions like
   1. See if some one is at home.
   2. Check if a item is present in a room of the house.
   3. Pick up objects. (Ex. trash)
4. Interact with the environment in a intelligent way.

## ROS moduls to acomplish the objective

In order to accomplish the objectives a set of ROS modules will need to be used.

### Simulation

For simulation it will be used Gazebo with a apartment like world comprise of a set of rooms, and the appropriated furniture.

The floor plant can be seen in the image bellow:

The gazebo simulation can be see here:

### Mapping and Navigation

For the mapping and navigation it will be used the gmapping module and the move base module, respectively.

### Perception

For the perception it will be use a set of two sensors. A laser scan for obstacle detection and mapping and a RGB camera for object recognition.

The RGB camera feed is passed to a script that makes use of the yolo library for the object recognition. The yolo weights file is not present in the repository since is a very large fille, however it can be downloaded.

### Manipulator

For the manipulator it will be use the moveit package. For the base it was used a already available version for the MOVEO robot that version can be found here:

[https://github.com/babakc/moveo_ros](https://github.com/babakc/moveo_ros)

### User Interface

To control the robot and send some basic commands we can use the created app using Tkinter. This interface can be seen bellow
![Alt text](images/app.jpeg)
*App interface*

## How to run the system

To lauch the gazebo simulation

```bash
roslaunch robutler_bringup gazebo.launch
```

To spawn the robot in the GAzebo world, and open the rviz

```bash
roslaunch robutler_bringup bringup.launch
```

After running this two commands it will be possible to see the following interfaces

![Alt text](images/Screenshot%20from%202023-01-28%2014-15-40.png)
*Rviz configuration exemple without the navigation stack*

![Alt text](images/Screenshot%20from%202023-01-28%2014-15-43.png)

*Close up on the robot model in Rviz*

![Alt text](images/Screenshot%20from%202023-01-28%2014-15-20.png)
*Exemple of the robot visual inside the simulated world*

This will launch the Moveit interface

```bash
roslaunch moveo_moveit_config demo.launch
```

Once this command is run the Moveit planing interface can be use to manipulate the robot position in the world.

![Alt text](images/Screenshot%20from%202023-01-28%2014-16-18.png)

*Example of a robot pose manipulation*

![Alt text](images/Screenshot%20from%202023-01-28%2014-23-19.png)

*Object recognition using YOLO*

## Available missions

## Future work

In a continuation of this work could be interesting to add a "voice" to our butler, this could be accomplish by making use of a text based artificial intelligence as well as text to speech algorithm.

Another improvement for the future could be to make use of the perception pipeline of the Moveit pkg, this would allow to add a depth sensor to the system and dynamically, from the point cloud data, add obstacles that would be use to improve the arm planning.

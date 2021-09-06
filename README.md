# RT2_Assignment1

This is the main package provided for the assignment (with an added launch file). It also includes a CoppeliaSim scene to interact with the ROS package.

When the user sends a start command, it continues to drive the robot to random goal positions. When a cancel request is sent, the user has to wait until the robot reaches its current goal, then, it stops.

This is the overall structure of the system:

![alt text](https://github.com/yaraalaa0/RT2_Assignment1/blob/main/graph_main.PNG?raw=true)

---------------------------------------------------------------------------------------------

To launch the system, run:

~~~
roslaunch rt2_assignment1 sim.launch
~~~
-------------------------------------------------------------------------------------------------

To launch the system with V-Rep, run:

~~~
roslaunch rt2_assignment1 sim_copp.launch
~~~

Then, start the simulation. 

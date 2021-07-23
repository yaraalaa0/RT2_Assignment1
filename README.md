# RT2_Assignment1

This is the action branch of the assignment. It performs the same behaviour as in the main branch, but with the difference of using action for sending the goal requests to the go_to_point server instead of service. This adds the ability to cancel the goal immediately and stop the robot once a cancel request has been received from the user. 

The system is composed of the simulation environment and four nodes:

- the node PositionServer which implements a random position service *(not modified from the main branch)*

- the node goToPointAc which implements an action server to drive a robot toward a goal in the environment, and checks periodically if a cancel request has been sent. If so, it preempts the current goal and stops the robot. *(has been modified from goToPoint node of the main branch)*

- the node FSM, which implements a server to start or stop the robot, and calls the PositionServer and goToPointAc to drive the robot. *(has been modified from the main branch)*

- the UserInterface node, which asks the user to start or stop the robot, and calls the service implemented in the FSM node. *(not modified from the main branch)*

Using this system, the robot can be stopped at anytime, without necessarily reaching the target. 

![alt text]https://github.com/yaraalaa0/RT2_Assignment1/blob/action/graph_assign1.PNG?raw=true)

## How to run the code:

put this package inside your ROS workspace (no additional packages are required)

In the root of your workspace, run: 

~~~
catkin_make
~~~

Then, to launch the simulation, run:

~~~
roslaunch rt2_assignment1_1 sim.launch
~~~

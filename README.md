# RT2_Assignment1

This is the action branch of the assignment. It performs the same behaviour as in the main branch, but with the difference of using action for sending the goal requests to the gotopoint server instead of service. This adds the ability to cancel the goal immediately and stop the robot once a cancel request has been received from the user. 

## How to run the code:

put this package inside your ROS workspace

In the root of your workspace, run: 

~~~
catkin_make
~~~

Then, run:

~~~
roslaunch rt2_assignment1_1 sim.launch
~~~

# RT2_Assignment1_2

The system here is supposed to perform the same behaviour as in the main branch. It drives the robot to successive random goal positions, and stops when the user send a stop command. However, this system is implemented partly on ROS and partly on ROS2 with a bridge between them to allow for their communication.

The package **rt2_assignment1** is implemented on ROS, and it is the main package of the assignment (implementing go_to_point as a service). 

On the other hand, the packages assignment_interfaces and rt2_assignment1_2 are implemented on ROS2. Their description is as the following:

## package assignment_interfaces:

contains the custom services: Command.srv , Position.srv , and RandomPosition.srv . It also contains the mappings file that maps these services to the services defined in the ROS package rt2_assignment1. This file is necessary for the ROS2-ROS1 bridge.

## package rt2_assignment1_2

This package contains the two cpp nodes of the main system: state_machine.cpp and position_service.cpp implemented on ROS2 as components. 

### 1- State Machine Node

The state machine node was first implemented as a ROS2 class, then, it was converted to a component. The StateMachine class has as a member the user_interface server, that changes the member flag "start" to true or false depending on the command received form the user. The StateMachine class has also as members the two clients to the position_server and the go_to_point server, with two member functions (call_server_rp() and call_server_p()) to call these services. When calling the services, the function async_send_request() is used with a callback function that gets executed once the service responce is received. When a responce to the service is received, the responce is printed on the screen and a flag related to the client is set to 1. This is to indicate receiving a response. There are two flags: flag_rp and flag_p. They correspond respectively to call_server_rp() (random position service) and call_server_p() (go_to_point service)

The StateMachine has a timer callback function (call_main()) that gets executed every 1 second. In call_main(), the value of the flag start is checked, and if true, the values of the flags flag_rp and flag_p are checked to check the current status of the robot. The robot can be waiting for a response from a service, in this case, call_main() doesn't do anything. If a response from the random_position service is received, the robot calls the call_server_p() function to drive the robot to the goal position. If the robot has reached the goal position, the robot calls the call_server_rp() to get a new random position. If the flag start is set to false, call_main() doesn't do anything.


### 2- Random Position Server Node

The random position server node was first implemented as a ROS2 class, then, it was converted to a component. The RandPosServer class has as a member the position_server server. The callback of this server gets the minimum and maximum limits sent in the request, and generates a random x, y, theta position. The, sends it as a response.


----------------------------------------------------------------------------------------------
## How to run the system:

- **Required Packages:**  

ros1_bridge ( https://github.com/ros2/ros1_bridge ). This package should be cloned inside the ROS2 workspace. Change the timeout of the bridge from 5 to 60 seconds. To do this, modify line 314 of the factory.hpp file, with: auto timeout = std::chrono::seconds(60);

- **Building the system:**

The package rt2_assignment1 should be put inside your ROS1 workspace. The packages assignment_interfaces and rt2_assignment1_2 should be put inside your RO2 workspace. The files: ros.sh, ros2.sh, ros12.sh, and bridge.sh should be put outside the workspaces. 

In the first shell, build your ROS1 workspace:

~~~
source ros.sh
cd ros_ws
catkin_make
~~~

In the second shell, build your ROS2 workspace, run:

~~~
source ros2.sh
cd ros2_ws
colcon build --symlink-install --packages-skip ros1_bridge
~~~

In the third shell, build the bridge, run:

~~~
source ros12.sh
cd ros2_ws
colcon build --packages-select ros1_bridge --cmake-force-configure
~~~

- **Launching the system:**

To launch the system, run the following command in the root folder:
~~~
./bridge.sh
~~~

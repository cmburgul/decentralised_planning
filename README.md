# decentralised_planning

# CHINMAY_BURGUL_INTERN

Approach :  

There are 121 agents initialised each standing on each node of a (11 X 11) nodes. Agent_id is a string named "agent_1". It ranges from "agent_1" till "agent_121". Initillay, agent_1 is on vertices (0, 0), agent_2 is on (1, 0), ..... agent_121 is on (10,10). The agent's posititon on the grid changes if the agent is given a goal to move. 

There are two types of coordinates vector coordinates and grid coordinates.

vector coordinates : These coordinates has a origin at the top-left corner of the grid map.
grid coordinates : These coordinates has a orgin at the bottom-left corner of the grid map.

The c++ std::vector< std::vector< node_class > > arranges the node_tempelates in the vector coordinates. The node_class has members which stores the grid coordinate values. And there are two functions which converts the grid coordinate values to vector coordinates and vice_versa named "con_vec_grid" and "con_grid_vec".

## Grid Initialization
The Grid Initialized in the std::vector< std::vector< node_class > > data type. Each class node contains the grid coordinate positions, hvalue, heuristics value and f value which are the parameters required for A* search algorithm.



## Steps to run /get_plan service using cpp file
```cpp
rosrun decentralised_planning get_plan_server 
```

rosrun decentralised_planning get_plan_client [agent_<id_number>] [start_x] [start_y] [start_yaw] [goal_x] [goal_y] [goal_yaw]
```cpp
rosrun decentralised_planning get_plan_client agent_1 2 3 0 10 10 0
```
After running server and client nodes to visualize the path in riz we need to run rviz and add **markers** with topic named **/map**
The position of the agent is updated to /agent_feedback topic.

## Steps to run /update_goal service using cpp file
```cpp
rosrun decentralised_planning update_goal_server 
```

rosrun decentralised_planning update_goal_client [agent_<id_number>] [start_x] [start_y] [start_yaw] [goal_x] [goal_y] [goal_yaw]
```cpp
rosrun decentralised_planning update_goal_client agent_1 2 3 0 10 10 0
```
After running server and client nodes to visualize the path in riz we need to run rviz and add **markers** with topic named **/map**
The position of the agent is updated to /agent_feedback topic

## Steps to launch /update_goal service launch file 
This launch files launches service server and serivce client. It takes 4 arguments :[agent_<id_number>] [start_x] [start_y] [start_yaw] [goal_x] [goal_y] [goal_yaw]

roslaunch decentralised_planning update_goal.launch [agent_id] [start_x] [start_y] [start_yaw] [goal.x] [goal.y] [goal.theta]

```cpp
roslaunch decentralised_planning get_plan.launch agent_id:=agent_1 start_x:=2 start_y:=3 start_yaw:=0 goal_x:=10 goal_y:=8 goal_yaw:=0
```

This launch file also launches rviz and shows the path


## Steps to launch unit_test_1 launch file 
This launch files launches service server and serivce client. It takes 4 arguments :[agent_<id_number>] [start_x] [start_y] [start_yaw] [goal_x] [goal_y] [goal_yaw]. But the launch file has default arguments and we need to just run it as below

```cpp
roslaunch decentralised_planning test_case_1.launch
```
This launch file will launch rviz, we need to add the marker and it shows the path.


## Steps to launch unit_test_2 launch file 
This launch files launches service server and serivce client. It takes 4 arguments :[agent_<id_number>] [start_x] [start_y] [start_yaw] [goal_x] [goal_y] [goal_yaw]. But the launch file has default arguments and we need to just run it as below

```cpp
roslaunch decentralised_planning test_case_2.launch
```
This launch file will launch rviz, we need to add the marker and it shows the path.


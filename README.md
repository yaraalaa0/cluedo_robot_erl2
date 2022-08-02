# cluedo_robot_erl2
Second assignment of Experimental Robotics Laboratory course, M.Sc. in Robotics Engineering, University of Genova, Italy

This is a ROS implementation of a robot agent playing a simplified Cluedo Game collecting hints and checking hypotheses. The robot's actions is planned using PDDL and executed using ROSPlan interfaces. The robot's knowledge is represented in OWL ontology that is being accessed using ARMOR client.  

The system was implemented and tested on the [docker image](https://hub.docker.com/repository/docker/carms84/exproblab) provided by Prof. Carmine Recchiuto, University of Genova, Italy

|       Author Name          | Student ID |      Email Address       |
| :------------------------: | :--------: | :----------------------: |
|     Yara Abdelmottaleb     |  5066359   |  [yara.ala96@gmail.com](mailto:yara.ala96@gmail.com)   |


## Introduction:

This is a ROS implementation of a robot agent playing a simplified Cluedo Game collecting hints and checking hypotheses. The agent goes randomly to one of four locations in the environment and can move its arm to one of two locations (low, high) to collect hints in the form of *(who, PERSON)*, *(where, PLACE)* and *(what, WEAPON)*. Collected hints are added to the ontology and after 3 hints are collected the agent goes to the center point to check if a correct hypothesis was found yet or not. The agent continues to explore the environment, collect hints, and check hypothesis until it finds the correct hypothesis.

## Component Diagram:

The software architecture of the system is composed of 13 main components: 

- **The knowledge base (ontology)**: this is the OWL ontology representing the current knowledge of the robot agent. In the beginning it contains the class definitions of `HYPOTHESIS`, `COMPLETE`, `INCONSISTENT`, `PERSON`, `PLACE`, and `WEAPON`, as well as the object properties definitions of *(who, PERSON)*, *(where, PLACE)*, and *(what, WEAPON)*. As the robot explores the environment, new individuals and proberty assertions are added to the ontology.
- **ARMOR**: the armor service responsible for connecting with the knowledge base for querying the ontology or updating it. It is fully implemented by [EmaroLab](https://github.com/EmaroLab/armor). In this project, it is mainly used by the ontology server for adding new hypotheses and hints, and querying the individuals of COMPLETE hypothesis class.
- **Ontology Server**: a ROS server implemented to handle the communication with AROMR. It can receive two types of service requests: adding hints to the ontology through the service `/add_hint` as a `erl2/Hint.h` message, and getting the list of current complete hypotheses through the service `/check_hyp_complete` as a `erl2/HypCompCheck.h` message. 
- **Simulation Oracle**: a ROS node representing the Oracle of the game. It continuously checks if the robot's end-effector link `cluedo_link` is within the area of one of the specific hint points. If yes, it publishes a random hint on the topic `/oracle_hint` as a `erl2/ErlOracle.h` message. The published hint can contain valid or non-valid values. It also offers a service `/oracle_solution` to send the correct hypothesis ID as a `erl2/Oracle.h` message.
- **ROSPlan**: a ROS package responsible for problem generation, planning, and plan execution given a problem and domain PDDL files.
- **Moveit**: Robotic manipulation platform responsible for planning and control of the robotic arm's joints to move the end-effector link from one point to the other. Two arm poses were defined for the robot: **h1**: where the end-effector is at height of 0.75 and **h2**: where the end-effector is at z height 1.25
- **Task Manager**: The main node that calls the ROSPlan services: problem generation, planning, parse plan, and plan dispatch. If the plan execution fails, it updates the current knowledge state based on the last dispatched action and re-plan.
- **AdjustInitHeihgt Action Interface**: The ROSPlan action interface for the PDDL action `adjust_init_height` responsible for adjusting the initial pose of the robotic arm. The target pose can be either `h1` or `h2`. It calls Moveit to move the end-effector link to the given initial target pose.
- **GoToWaypoint Action Interface**: The ROSPlan action interface for the PDDL action `goto_waypoint` responsible for moving the robot base from one waypoint to another. Waypoints can be one of five: `wp0`: (0 , 0), `wp1`: (2.4 , 0), `wp2`: (0 , 2.4), `wp3`: (-2.4 , 0), and `wp4`: (0 , -2.4). It sends the goal waypoint to the `Go To Point` action server and waits until it is reached.
- **Go To Point Action Server**: a ROS action server that drives the robot towards a given target pose by offering an action service `reaching_goal` that accepts action messages in the form of `erl2/PlanningAction.h`. The node was modified to adjust the final yaw angle of the robot depending on the given target waypoint to make it face the wall of the environment . This is to facilitate reaching the hint areas with the robotic arm.
- **GetHint Action Interface**: The ROSPlan action interface for the PDDL action `get_hint` responsible for receiving hints from the oracle through the topic `/oracle_hint` and checking the validity of the received hint. If there is a newly received hint and it is valid (the key and value are not empty nor equal -1), it calls the service `/add_hint` to add the received hint to the ontology.
- **MoveArm Action Interface**: The ROSPlan action interface for the PDDL action `move_arm` responsible for moving the robotic arm to a target pose. The target pose can be either `h1` or `h2`. It calls Moveit to move the end-effector link to the given target pose.
- **CheckHypothesisCorrect Action Interface**: The ROSPlan action interface for the PDDL action `check_hypothesis_correct` responsible for checking if one of the collected hypotheses is the correct one or not. It calls the service `/check_hyp_complete` to get the list of complete hypotheses IDs, and it calls the service `/oracle_solution` to get the correct hypothesis ID. It, then, checks if one of the complete hypotheses is the correct one or not.


![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/Cluedo2_comp_diag.jpg?raw=true)


## State Diagram:

The agent has four possible states:
- **GoingToRandomPlace:** the robot is going to a random waypoint for exploration
- **GettingHints:** the robot is checking for hints in the place it is currently in
- **GoingToCenter:** the robot is going to the center waypoint
- **CheckingHypothesis:** the robot is checking whether one of its current collected hypotheses is the correct one

There are, also, four possible events (state transitions):
- **reached:** indicating that the robot reached its target position
- **got a hint:** indicating that the robot received a hint (whether it is valid or not)
- **collected 3 hints:** indicating that the robot collected 3 hints
- **hyp_non_correct:** indicating that the robot checked the current hypotheses and none of them was correct.
 

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/cluedo2_state_diag.jpg?raw=true)

## Sequence Diagram:
A possible temporal sequence of the program can go as follows:

1. `Task Manager` calls `ROSPlan` for problem generation, planning, and plan execution given the PDDL domain and problem files
2. `ROSPlan` sends execute command to the `AdjustInitHeight` action interface
3. `AdjustInitHeight` sends the target arm pose to Moveit
4. `AdjustInitHeight` returns `True` to `ROSPlan`
5. `ROSPlan` sends execute command to the `GoToWaypoint` action interface
6. `GoToWaypoint` sends the corresponding (x,y) position of the named waypoint to `go_to_point` action server.
7. `go_to_point` drives the robot towards the received goal and sends a response to `GoToWaypoint` after reaching the goal.
8. `GoToWaypoint` returns `True` to `ROSPlan`
9. `ROSPlan` sends execute command to the `GetHint` action interface
10. `GetHint` receives a new hint from `Simulation Oracle`
11. `GetHint` sends a service request to `Ontology Server` to add the hint to the ontology
12. `Ontology Server` returns `True` after it adds the hint
13. `ROSPlan` returns `True` to `ROSPlan`
14. `ROSPlan` sends execute command to the `MoveArm` action interface
15. `MoveArm` sends the target arm pose to Moveit
16. `MoveArm` returns `True` to `ROSPlan`
17. `ROSPlan` sends execute command to the `GetHint` action interface
18. `GetHint` doesn't receive any hint. So, it returns `False` to `ROSPlan`
19. `ROSPlan` returns goal success = `False` to `Task Manager`
20. `Task Manager` checks the last dispatched action before failure and updates the current state to `ROSPlan`
21. `Task Manager` calls `ROSPlan` for re-planning and execution
22.  `ROSPlan` sends execute command to the `GoToWaypoint` action interface
23.  `GoToWaypoint` sends the corresponding (x,y) position of the named waypoint to `go_to_point` action server.
24. `go_to_point` drives the robot towards the received goal and sends a response to `GoToWaypoint` after reaching the goal.
25. `GoToWaypoint` returns `True` to `ROSPlan`
26. `ROSPlan` sends execute command to the `GetHint` action interface
27. `GetHint` receives a new hint from `Simulation Oracle`
28. `GetHint` sends a service request to `Ontology Server` to add the hint to the ontology
29. `Ontology Server` returns `True` after it adds the hint
30. `GetHint` returns `True` to `ROSPlan`
31. `ROSPlan` sends execute command to the `MoveArm` action interface
32. `MoveArm` sends the target arm pose to Moveit
33. `MoveArm` returns `True` to `ROSPlan`
34. `ROSPlan` sends execute command to the `GetHint` action interface
35. `GetHint` receives a new hint from `Simulation Oracle`
36. `GetHint` sends a service request to `Ontology Server` to add the hint to the ontology
37. `Ontology Server` returns `True` after it adds the hint
38. `GetHint` returns `True` to `ROSPlan`
39. `ROSPlan` sends execute command to the `GoToWaypoint` action interface to go to the center point
40. `GoToWaypoint` sends the corresponding (0,0) position of `wp0` to `go_to_point` action server.
41. `go_to_point` drives the robot towards the received goal and sends a response to `GoToWaypoint` after reaching the goal.
42. `GoToWaypoint` returns `True` to `ROSPlan`
43. `ROSPlan` sends execute command to the `CheckHypCorrect` action interface
44. `CheckHypCorrect` sends a service request to `Ontology Server` to get the list of collected complete hypotheses IDs in the ontology
45. `Ontology Server` replies with the list of IDs of collected complete hypotheses
46. `CheckHypCorrect` sends a service request to `Simulation Oracle` to get the ID of the correct hypothesis
47. `Simulation Oracle` replies with the ID of the correct hypothesis
48. `CheckHypCorrect` checks if one of the completed hypotheses is the correct one
49. If yes, `CheckHypCorrect` returns `True` to `ROSPlan`
50. `ROSPlan` returns goal success = `True` to `Task Manager`

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/cluedo2_seq_diag.jpg?raw=true)

## Installation and Running Procedures:

To run the program, you need first to install [ARMOR](https://github.com/EmaroLab/armor) in your ROS workspace.

Then, you need to adapt the code in armor_py_api scripts to be in Python3 instead of Python2:
  - add "from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError" in armor_client.py
  - replace all "except rospy.ServiceException, e" with "except rospy.ServiceException as e"
  - modify line 132 of armor_query_client with: "if res.success and len(res.queried_objects) > 1:"

Add the path of the armor modules to your Python path:
```
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```
Download this repository to your workspace. Then, build it

```
catkin_make
```

Place `cluedo_ontology.owl` file on your desktop (or on any other place, but you need to specify the path inside [ontology_server.py](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2/scripts/ontology_server.py))

To launch the program, run the following commands in order on four terminal tabs:
- launch ROSplan with the action interfaces: 
```
roslaunch erl2 rosplan_cluedo.launch
```
- Launch ARMOR:
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
- Launch the simulation, ontology server, and go_to_point action server:
```
roslaunch erl2 assignment.launch
```
- Run the task manager to start the game:
```
rosrun erl2 task_manager.py
```
The received hints are displayed on the first terminal. The plan success result is displayed on the fourth terminal.

## Result:
**Following are screenshots of the running simulation in successive timesteps:**

1. The simulation starts, the robot is in initial random pose

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/initial_pose.PNG?raw=true)

2. First action in the plan `AdjustInitHeight` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a1.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p1.PNG?raw=true)

3. Action `GoToWaypoint(wp0,wp1)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a2.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p2.PNG?raw=true)

4. Action `GetHint` is executed.

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a3.PNG?raw=true)

5. No hints received. So, the task_manager updates the current state and re-plan

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/re-planning__1.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/re-planning4.PNG?raw=true)

6. Action `GoToWaypoint(wp1,wp2)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a5.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p3.PNG?raw=true)

7. Action `GetHint` is executed.

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a6.PNG?raw=true)

8. No hints received. So, the task_manager updates the current state and re-plan

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/re-planning__2.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/re-planning7.PNG?raw=true)

9. Action `GoToWaypoint(wp2,wp3)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a8.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p4.PNG?raw=true)


10. Action `GetHint` is executed.

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a9.PNG?raw=true)


11. No hints received. So, the task_manager updates the current state and re-plan

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/re-planning__3.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/re-planning10.PNG?raw=true)

12. Action `GoToWaypoint(wp3,wp4)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a11.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p5.PNG?raw=true)


13. Action `GetHint` is executed. There is a received hint.

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a12.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/hint12.PNG?raw=true)

14. Action `GoToWaypoint(wp4,wp1)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a13.PNG?raw=true)


15. Action `MoveArm(h1,h2)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a14.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p6.PNG?raw=true)


16. Action `GetHint` is executed. There is a received hint.

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a15.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/hint15.PNG?raw=true)


17. Action `GoToWaypoint(wp1,wp2)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a16.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p7.PNG?raw=true)


18. Action `GetHint` is executed. There is a received hint.

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a17.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/hint17.PNG?raw=true)


19. The robot collected 3 hints. So, it goes to the center point. Action `GoToWaypoint(wp2,wp0)` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a18.PNG?raw=true)

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/p8.PNG?raw=true)


20. Action `CheckHypCorrect` is executed

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/a19.PNG?raw=true)


21. When the robot finds the correct hypothesis, the output can be like this:

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/final20.PNG?raw=true)


22. Plan is successful

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/erl2_images/final_result.PNG?raw=true)


## System Features & Working Assumptions:
- There are four possible (x,y) locations for receiving hints and they are: (3,0), (0,3), (-3,0), (0,-3).
- When collecting hints, the robot doesn't stand exactly at the previously defined locations. However, it stands at the locations: (2.4,0), (0,2.4), (-2.4,0), (0,-2.4) respectively. This is to give space to the robot's arm to reach the previously defined hints locations with its end-effector.
- There are two possible heights for the hints and they are: 0.75 and 1.25. Two arm poses, `low` and `high`, are defined to reach these two heights using Moveit. 
- The robot aligns itself to be facing the wall depending on its location. This is done to facilitate reaching the `low` and `high` hints points with the robot's arm.
- The `adjust_init_height` action is necessary because at the beginning of the simulation the robot's arm gets initialized to a random pose (It is a strange behaviour since the initial pose was already defined in moveit to be the `low` pose).
- The robot has to go regularly to the center point to check if it has collected a correct hypothesis or not. If it hasn't collected a correct hypothesis yet, the robot starts a new round of exploring waypoints and collecting hints.
- The robot doesn't go to the center point unless it has collected at least 3 hints (from the beginning of this round) whether these hints are valid or not.
- When checking the correctness of the collected hypotheses, the robot checks all the collected `complete` hypotheses, whether they are consistent or not.
- The action `get_hint` fails only when the robot doesn't receive any hint (`cluedo_link` is not within a hint area). Receiving a non-valid hint doesn't cause the action to fail. However, it doesn't add this non-valid hint to the ontology.
- When the action `get_hint` fails, the plan execution fails and the task manager updates the knowledge base of ROSPlan with the current state (in this case, the number of collected hints and the explored waypoints from the beginning of the round are kept unchanged) and send a re-planning request to ROSPlan.
- When the action `check_hyp_correct` fails, the plan execution fails and the task manager updates the knowledge base of ROSPlan with the initial state of the system where all waypoints are unexplored and the number of collected hints is 0. Then, the task manager sends a re-planning request to ROSPlan.
- The robot keeps going to waypoints, getting hints, and checking hypotheses until the plan is successful. This happens only when the action `check_hyp_correct` returns true.


## System's Limitations:
- The robot is constrained to the five defined waypoints: `wp0`: (0,0), `wp1`: (3,0), `wp2`: (0,3), `wp3`: (-3,0), `wp4`: (0,-3). Changing the hints waypoints or adding new ones may cause the system to fail.
- The robot is constrained to the two defined heights: `h1`: 0.75 and `h2`: 1.25. Changing the hints heights will cause the system to fail.
- The system was tested successfully on the provided simulation environment. Changing the environment by, for example, changing the walls configuration may cause the system to fail.

## Possible Improvements:
- Implement a real simulation environment for the game where the robot has to go around the rooms and infer the hints by analyzing, for example, images coming from camera sensors. 
- Implement a computer vision module that can infers the hint by analyzing images of the room coming from the camera sensor.
- Allow the hints waypoints and heights to be undefined to the robot and the robot has to develop a strategy to explore different locations and heights to collect hints. This is to make the situation more realistic. 

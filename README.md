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
- **Ontology Server**: a ROS server implemented to handle the communication with AROMR. It can receive two types of service requests: adding hints to the ontology, and getting the list of current complete hypotheses. 
- **Simulation Oracle**: a ROS node representing the Oracle of the game. It continuously checks if the robot's end-effector link `cluedo_link` is within the area of one of the specific hint points. If yes, it publishes a random hint on the topic `/oracle_hint`. The published hint can contain valid or non-valid values. It also offers a service `oracle_solution` to send the correct hypothesis ID.
- **ROSPlan**: a ROS package responsible for problem generation, planning, and plan execution given a problem and domain PDDL files.
- **Moveit**: Robotic manipulation platform responsible for planning and control of the robotic arm's joints to move the end-effector link from one point to the other. Two arm poses were defined for the robot: **h1**: where the end-effector is at height of 0.75 and **h2**: where the end-effector is at z height 1.25
- **Task Manager**: The main node that calls the ROSPlan services: problem generation, planning, parse plan, and plan dispatch. If the plan execution fails, it updates the current knowledge state based on the last dispatched action and re-plan.
- **AdjustInitHeihgt Action Interface**: The ROSPlan action interface for the PDDL action `adjust_init_height` responsible for adjusting the initial pose of the robotic arm. The target pose can be either `h1` or `h2`. It calls Moveit to move the end-effector link to the given initial target pose.
- **GoToWaypoint Action Interface**: The ROSPlan action interface for the PDDL action `goto_waypoint` responsible for moving the robot base from one waypoint to another. Waypoints can be one of five: `wp0`: (0 , 0), `wp1`: (2.4 , 0), `wp2`: (0 , 2.4), `wp3`: (-2.4 , 0), and `wp4`: (0 , -2.4). It sends the goal waypoint to the `Go To Goal` action server and waits until it is reached.
- **Go To Goal Action Server**: a ROS action server that drives the robot towards a given (x,y) goal. The node was modified to adjust the final yaw angle of the robot to be facing the wall of the environment depending on the given target waypoint. This is to facilitate reaching the hint areas with the robotic arm.
- **GetHint Action Interface**: The ROSPlan action interface for the PDDL action `get_hint` responsible for receiving hints from the oracle through the topic `/oracle_hint` and checking the validity of the received hint. If there is a newly received hint and it is valid (the key and value are not empty nor equal -1), it calls the service `/add_hint` to add the received hint to the ontology.
- **MoveArm Action Interface**: The ROSPlan action interface for the PDDL action `move_arm` responsible for moving the robotic arm to a target pose. The target pose can be either `h1` or `h2`. It calls Moveit to move the end-effector link to the given target pose.
- **CheckHypothesisCorrect Action Interface**: The ROSPlan action interface for the PDDL action `check_hypothesis_correct` responsible for checking if one of the collected hypotheses is the correct one or not. It calls the service `/check_hyp_complete` to get the list of complete hypotheses IDs, and it calls the service `/oracle_solution` to get the correct hypothesis ID. It, then, checks if one of the complete hypotheses is the correct one or not.


![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/Cluedo2_comp_diag.jpg?raw=true)


## State Diagram:

The agent has four possible states:
- **GoToRandomRoom:** the robot is going to a random room for exploration
- **LookForHints:** the robot is looking for hints in the place it is currently in
- **GoToOracle:** the robot is going to the oracle place
- **CheckHypothesis:** the robot is checking whether its current collected hypothesis is true or not

There are, also, four possible events (state transitions):
- **reached:** indicating an event that the robot reached its target position
- **hyp_non_comp:** indicating an event that the robot checked the current hypothesis and found that it is not complete yet
- **hyp_comp:** indicating an event that the robot checked the current hypothesis and found that it is complete
- **false:** indicating that the oracle checked the current hypothesis and found that it is false.
 

![alt text](https://github.com/yaraalaa0/cluedo_robot_erl2/blob/main/cluedo2_state_diag.jpg?raw=true)

## Sequence Diagram:
The temporal sequence of the program goes as follows:

1. The state machine requests a random room from the map server, and receives the *(x,y)* position
2. it sends the room coordinates to the motion controller and waits until the robot reaches the target
3. it sends the current hypothesis ID to the oracle and receives a random hint
4. it adds the hint to the ontology
5. it checks if the current hypothesis is complete or not (by querying the members of the `COMPLETE` class in the ontology)
6. if the current hypothesis is not complete yet, go to step 1
7. if the current hypothesis is complete, the state machine requests the *(x,y)* position of the oracle from the map server
8. it sends the oracle coordinates to the motion controller and waits until the robot reaches the target
9. it sends the current hypothesis ID to the oracle to check if it is correct or not
10. if the sent hypothesis ID is not correct, generate a new random integer (not previously selected) from 1 to 10 to be the current hypothesis ID and go to step 1.
11. if the sent hypothesis ID is correct, end the program.

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

Place `cluedo_ontology.owl` file on your desktop (or on any other place, but you need to specify the path inside [state_machine.py](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo/scripts/state_machine.py))

To launch the program, run the following commands on different terminal tabs:
```
roscore
```
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
```
roslaunch cluedo cluedo.launch`
```
The robot behaviour is continuously logged on the third terminal.

To display the states:
```
rosrun smach_viewer smach_viewer.py
```

## Result:
**Following are screenshots of the terminal logs in successive timesteps while running the program:**

1. The program selects a hypothesis ID and starts with the state GoToRandomRoom. After reaching, transition to LookForHints
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st1.PNG?raw=true)

2. The hint is displayed. The current hypothesis is checked if it is complete or not. It is not complete yet. So, the agent transition to GoToRandomRoom 
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st2.PNG?raw=true)

3. The agent goes to a random room and look for hints as before. The hint is displayed and the current hypothesis is checked if it is complete or not. It is not complete yet. So, the agent transition to GoToRandomRoom 
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st3.PNG?raw=true)

4. The agent goes to a random room and look for hints as before. Here, the agent doesn't get any hint from this room. The current hypothesis is checked as before. It is not complete yet. So, the agent transition to GoToRandomRoom 
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st4.PNG?raw=true)

5. The agent goes to a random room and look for hints. The hint is displayed and the current hypothesis is checked if it is complete or not. Now, it is complete. So, the agent transition to GoToOracle state.
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st5.PNG?raw=true)

6. The agent goes to the oracle. After reaching, transition to CheckHypothesis
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st6.PNG?raw=true)

7. The hypothesis is displayed. The result of checking is FALSE. So, new hypothesis ID is selected, and the agent transition to GoToRandomRoom as before.
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st7.PNG?raw=true)

8. The agent repeates the above steps until the result of checking the hypothesis is CORRECT. In that case, the current ontology is saved and the user is asked to terminate the program.
![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/st8.PNG?raw=true)

## Working Assumptions:
- The hints are selected randomly from the dictionary stored in the oracle based on the hypothesis ID
- In the dictionary of hints stored in the oracle, each hypothesis has four hints. Three are of the types: *(who,PERSON)*, *(what,WEAPON)*, *(where,PLACE)*. The fourth one is the empty hint.
- The robot can get one hint at a time from the same room. Obviously, this hint can be empty. 
- The hint obtained at each room doesn't have to be related to the type of room itself. For example, the agent can receive a hint (where, bathroom) in the kitchen.
- Whenever a hint is sent to the robot, it is deleted from the dictionary in order not to be sent again. 
  > This is only true for the three types of hints. The empty hint can be sent multiple times.
- The robot only goes to the oracle when the current hypothesis is `COMPLETE`
- The new current hypothesis ID is selected randomly from the list of possible hypothesis IDs. 
- The list of possible hypothesis IDs range from 1 to 10 and they can be modified in the beginning of [state_machine.py](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo/scripts/state_machine.py) The correct hypothesis ID is a number within this range.
- Whenever an ID is selected, it cannot be selected again.
- The environment map of obstacles is stored in the motion controller action server for path planning purposes

## Possible Improvements:
- Allow a hypothesis not to necessarily contain all three types of hints. It can contain only one or two types. It can be inconsistent.
- Implement a robot model and a game environment to make it more entertaining
- Implement a real motion controller for the robot instead of the simple waiting function.
- Implement a computer vision module that can infers the hint by analyzing images of the room coming from the camera sensor.
- For checking the correctness of the hypothesis, the robot should send the whole hypothesis elements instead of the hypothesis ID. Based on the hypothesis elements (hints), the oracle should decide if it is correct or not. 

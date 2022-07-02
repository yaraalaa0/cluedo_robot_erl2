# cluedo_robot_erl2
Second assignment of experimental robotics laboratory course, University of Genova, Italy

This is a ROS implementation of a robot agent playing a simplified Cluedo Game. The robot's actions is planned using PDDL and executed using ROSPlan interfaces. The robot's knowledge is represented in OWL ontology that is accessed using ARMOR client.  

The system was implemented and tested on the [docker image](https://hub.docker.com/repository/docker/carms84/exproblab) provided by Prof. Carmine Recchiuto, University of Genova, Italy

|       Author Name          | Student ID |      Email Address       |
| :------------------------: | :--------: | :----------------------: |
|     Yara Abdelmottaleb     |  5066359   |  [yara.ala96@gmail.com](mailto:yara.ala96@gmail.com)   |


## Introduction:

This is a ROS implementation of an agent playing a simplified Cluedo Game. The agent goes to random rooms and collects hints in the form of *(who, PERSON)*, *(where, PLACE)* and *(what, WEAPON)*. When the agent's current working hypothesis is `COMPLETE` (that means it contains all three types of hints), it goes to a place called the Oracle where the hypothesis is checked. The agent continues to explore the environment, collect hints, and check hypothesis until it finds the correct hypothesis.

## Component Diagram:

The software architecture of the system is composed of six main components: 

- **The knowledge base (ontology)**: this is the OWL ontology representing the current knowledge of the robot agent. In the beginning it contains the class definitions of `HYPOTHESIS`, `COMPLETE`, `INCONSISTENT`, `PERSON`, `PLACE`, and `WEAPON`, as well as the object properties definitions of *(who, PERSON)*, *(where, PLACE)*, and *(what, WEAPON)*. As the robot explores the environment, new individuals and proberties assertions are added to the ontology.
- **ARMOR**: the armor service responsible for connection with the knowledge base for querying the ontology or manipulating it. It is fully implemented by [EmaroLab](https://github.com/EmaroLab/armor). In this project, it is mainly used by the state machine for adding new hypotheses and hints, and querying the individuals of COMPLETE hypothesis class.
- **State Machine**: this is the state manager of the robot. It is responsible for controlling the transitions between different robot states (*GoToRandomRoom*, *LookForHints*, *GoToOracle*, and *CheckHypothesis*). It also implements the robot behaviour in each state. It communicates with the other servers through different ROS messages. The ROS messages and parameters are indicated in the component diagram.
- **Map Server**: this component holds the *(x, y)* position of all rooms in the map. The service request is composed of a flag `bool randFlag` indicating whether the server should return a random room position `randFlag = True` or the oracle postion `randFlag = False`
- **Motion Controller**: this is the action server responsible for driving the robot towards a target *(x,y)* position. For now, it is implemented as a simple waiting function for 5 seconds and it always returns True. 
- **Oracle**: this is the component holding the dictionary of possible hypothesis IDs along with their hints. Each hypothesis ID has 4 hints: one is the empty hint, and the others are *(who, PERSON)*, *(what, WEAPON)*, and *(where, PLACE)*. Whenever the oracle receives a request for the service `/hint` with a specific ID, it returns back a random hint corresponding to this ID in the form `string arg1` and `string arg2` (where `arg1` is 'who', 'what', or 'where', and `arg2` is the name). Once a hint is sent, it is deleted from the dictionary in order not to be sent again. This component also holds the correct hypothesis ID. Whenever it receives a request for the service `/check_hyp` with a specific ID, it returns back whether this is the correct hypothesis ID or not.

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
 

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/cluedo_state_diag.PNG?raw=true)

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

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/program_images/cluedo_seq_diag2.PNG?raw=true)

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

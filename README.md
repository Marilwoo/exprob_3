# Experimental Robotics Laboratory - Third assignment
Maria Luisa Aiachini - 4375373

### General introduction
This project simulates a simple version of cluedo. The robot is inside a labyrinth made of 6 rooms that have known positions:(-4,-3)(-4,2)(-4,7)(5,-7)(5,-3)(5,1). The robot needs to go inside each room and search for hints. The hints are made by aruco markers: there are 5 markers in each room. The markers can have three different positions: placed at the top of the walls and on the floor vertically or horizontally. Once the robot has found a complete and consistent hypothesis, or it has visited all the rooms it goes to the home position (0,-1) to check if the winning hypothesis is found. If so the game ends; if not it will start again searching.

### Expected behavior
The robot once the simulation has started execute the following steps:
- Randomly chooses which room to visit
- Moves towards that room
- Once the room is reached it starts to turn in on itself to look for markers
- Checks if it has any complete and consistent hypothesis
- If yes goes to home position to check if it is the winning one
- If not, randomly chooses the next room to visit
- Once all the rooms are visited once it goes to the center to check the hypotheses
- If winning hypothesis is found the game ends
- If not, the behavior starts from the beginning



### Software architecture
- UML
- Temporal
- Packages description

#### ROS msgs
Ros messages in erl2 package
- [ErlOracle.msg](https://github.com/CarmineD8/erl2/blob/main/msg/ErlOracle.msg): used by Marker.srv, contains the three parameters that make up the hint.
```
int32 ID
string key
string value
```

#### ROS srv
- Ros services in exprob_3 package:
	- [Hints.srv](https://github.com/Marilwoo/exprob_3/blob/master/srv/Hints.srv): used by `/hint_list`. Used to send the hypotheses list.
	```
	---
	string[] hint_0
	string[] hint_1
	string[] hint_2
	string[] hint_3
	string[] hint_4
	string[] hint_5
	```
	
	- [Marker.srv](https://github.com/Marilwoo/exprob_3/blob/master/srv/Marker.srv): used by `/oracle_hint`. Used to send the hint corresponding to the marker id requested.
	```
	int32 markerId
	---
	erl2/ErlOracle oracle_hint
	```
	
- Ros services in erl2 package:
	-[Oracle.srv](https://github.com/CarmineD8/erl2/blob/main/srv/Oracle.srv): used by `/oracle_solution`. Used to send the winning ID when requested.
	```
	---
	int32 ID
	```
	
### Installation and how to run
For this project you need to clone in your ros_ws ROS workspace these packages:
- [Aruco_ros package](https://github.com/CarmineD8/aruco_ros)
- [slam_gmapping package](https://github.com/CarmineD8/SLAM_packages/tree/noetic)
- [erl2 package](https://github.com/CarmineD8/erl2/)
- [exprob_3 package](https://github.com/Marilwoo/exprob_3)

To build the code run:
```
catkin_make
```
Launch the simulation with:
```
roslaunch exprob_3 simulation.launch
```
**To get rid of unwanted warning messages launch the simulation with:**
```
roslaunch exprob_3 simulation.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)

```

### Working description: screenshots of running program


### System features
- Multiple cameras

### System limitation
- Molto lento
- Non prende tutti gli hint alla prima -> col passare del tempo e la randomicità delle stanze arriva alla soluzione

### Possible improvements


### Contacts
Maria Luisa Aiachini - 4375373

4375373@studenti.unige.it

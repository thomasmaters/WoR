## Robot Simulation ##

### Package structure ###
* config (Hier zijn config file voor rviz te vinden)
* launch (Folder voor de launch file(s) om de applicatie makkelijk te starten)
* msg (Ros messages)
* src (Source code van het project)
* urdf (De urdf modelen voor rviz)

### Build requirements ###
Om dit project te bouwen zijn de volgende vereisten nodig:

- ROS Kinetic(minimaal)
- Boost v1.58
- OpenCV 2
- Gnu 6.2
- thread
- chrono
- regex

### Build instructions ###
Om het project te builden moeten de volgende commando's uitgevoerd worden: 

```
cd ../
catkin_make
```

### Run instructions ###
Om de demo te starten voor de volgende commando's uit in uw terminal:

```
source ./devel/setup.bash
roslaunch robotsimulation default.launch demo:=true
rosrun robotsimulation cli_node
```

Om handmatige de simulatie aan te sturen dient uw de volgende commando uit te voeren:

```
source ./devel/setup.bash
roslaunch robotsimulation default.launch
rostopic pub /ssc32u_topic robotsimulation/ssc32u_command "command: '#0P1500T1500'" 
```
'#0P1500T1500' dient uw te vervangen met een geldig ssc32u commando.

Wilt uw de arm besturen met een GUI type de volgende commando's in uw terminal

```
source ./devel/setup.bash
roslaunch robotsimulation default.launch gui:=true
```

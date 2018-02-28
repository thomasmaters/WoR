## Robot Simulation ##

### Package structure ###
|-config (Hier zijn config file voor rviz te vinden)
|-launch (Folder voor de launch file(s) om de applicatie makkelijk te starten)
|-msg (Ros messages)
|-src (Source code van het project)
|-urdf (De urdf modelen voor rviz)

### Build requirements ###
Om dit project te bouwen zijn de volgende vereisten nodig:

-ROS Kinetic(minimaal)
-Boost v1.58
-OpenCV 2
-Gnu 6.2
-thread
-chrono
-regex

### Build instructions ###
Om het project te builden moeten de volgende commando's uitgevoerd worden: 

@code

cd ../
catkin_make

@endcode

### Run instructions ###
Om de demo te starten voor de volgende commando's uit in uw terminal:

@code

source ./devel/setup.bash
roslaunch robotsimulation default.launch demo:=true
rosrun robotsimulation cli_node

@endcode

Om handmatige de simulatie aan te sturen dient uw de volgende commando uit te voeren:

@code
source ./devel/setup.bash
roslaunch robotsimulation default.launch
rostopic pub /ssc32u_topic robotsimulation/ssc32u_command "command: '#0P1500T1500'" 
@endcode
'#0P1500T1500' dient uw te vervangen met een geldig ssc32u commando.

Wilt uw de arm besturen met een GUI type de volgende commando's in uw terminal

@code
source ./devel/setup.bash
roslaunch robotsimulation default.launch gui:=true
@endcode

HET IS NIET AAN TE RADEN OM MET gui:=true en demo:=true TEGELIJK AAN TE HEBBEN, HIERDOOR WEET DE SIMULATIE NIET WELKE WAARDES VAN DE JOINTS JUIST ZIJN.

### Criteria ###
 
|Requirement|Punten|MoSCoW|Afgerond|Onderdeel
---|---|---|---|---
PA01 | 3 | S | X | 
PA02 | 3 | M | X |
PA03 | 3 | S | X |
VS01 | 3 | M | X | 
VS02 | 5 | M | X | 
VS03 | 3 | C | X | 
VS04 | 2 | S | X |
VS05 | 2 | S |   |
VC01 | 4 | S | X |
VC02 | 1 | C |   |
VC03 | 4 | C | X |
VC04 | 5 | S | X | 
VC05 | 2 | C |   | 
VC06 | 5 | M | X |
VC07 | 2 | C | X |
VC08 | 4 | M | X | 
VC09 | 2 | S |   |
VC10 | 1 | C |   |
VC11 | 1 | C |   |
VC12 | 1 | C |   |
VC13 | 2 | C | X |
VC14 | 2 | C |   |
DI01 | 2 | S | X |
DI02 | 2 | C |   |
DI03 | 2 | C | X | 
DI04 | 2 | S | X | 
DM01 | 2 | M | X |
DM02 | 4 | M | X |
DM03 | 4 | M | X |
DM04 | 4 | M | X |
DD01 | 2 | M |   |
DD02 | 4 | S | X |
DD03 | 4 | S |   |
DD04 | 5 | S |   |
DD05 | 3 | S |   |

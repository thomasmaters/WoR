## Robot Application ##

### cppcheck warnings ###
De matrix classen kan ExplicitConstructor meldingen geven voor 3 verschillende constructors. Worden deze constructors echter niet explicit gemaakt worden voor de werking van de applicatie.

### valgrind warnings ###
Tijdens het uitvoeren van de applicatie met Valgrind kunnen er verscheidende ros errors geproduceerd worden. Dit is echter een probleem van ros zelf en heeft geen invloed op de werking van de applicatie.

### -Wconversion warning ###
Tijdens het compileren met '-Wconversion' kan het voorkomen dat er conversion warnings komen uit ros/time.h. Dit is echter een probleem van ros zelf en heeft geen invloed op de werking van de applicatie.

### Compilatie benodigtheden ###

- GCC 6.3
- OpenCV 2.0
- Ros Lunar 1.13.6
- C++ 11

### Compilatie instructies ###

Plaats deze package in een catkin workspace.
Voer vervolgens het volgende commando uit in een bash shell.

```
catkin_make --pkg robotapplication
```

### Run instructies ###
Nadat de package is gecompileerd kan hij gestart worden met de volgende commando's uit te voeren in een bash shell.

```
source ./devel/setup.bash
rosrun robotapplication motion_control_node
```

of

```
source ./devel/setup.bash
roslaunch robotapplication default.launch
```

Indien de packages robotarminterface, robotvision & robotsimulation ook aanwezig zijn en gecompileerd:

```
source ./devel/setup.bash
roslaunch robotapplication default.launch full:=true
```
## Robot Application ##

### cppcheck warnings ###
De matrix classen kan ExplicitConstructor meldingen geven voor 3 verschillende constructors. Worden deze constructors echter niet explicit gemaakt worden voor de werking van de applicatie.

### -Wconversion warning ###
Tijdens het compileren met '-Wconversion' kan het voorkomen dat er conversion warnings komen uit ros/time.h. Dit is echter een probleem van ros zelf en heeft geen invloed op de werking van de applicatie.

### Compilatie benodigtheden ###

- GCC 6.3
- OpenCV 2.0
- Ros Lunar 1.13.6
- C++ 11



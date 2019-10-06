
This module makes use of a MPU9250 board installed in a OttoDIY. It uses the magnetometer to get the robot's heading and stick to it walking for a certain number of paces and then turning around and coming back (roughly) to the point it started.
  
Tweaking the hard coded angles of one step of the right and left turns will be required to make this work on any individual OttoDIY. See straight, right, left conditional statement in loop() below.

Angle values in the walk() and turn() routines of Otto.cpp and Otto.h have been modified to make those functions work reasonably well for my specific OttoDIY.

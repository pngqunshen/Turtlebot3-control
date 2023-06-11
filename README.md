# Turtlebot Path Planning and Control

Path planning and control of turtlebot3 for EE4308 Autonomous Robot Systems project

## Features
1. Theta\* planner
2. Quintic Hermite trajectory planner
3. BFS goal replan to avoid inaccessible goals
4. Bidirectional motion

## Instructions

1. Move the EE4308\_turtle folder to the turtlebot and build by calling make.sh
```
./make.sh
```
2. Set up turtlebot networking http://wiki.ros.org/Robots/TurtleBot/Network%20Setup
3. Run the bringup.sh and run.sh scripts on the remote device on different terminal
```
./bringup.sh
```
```
./run.sh
```

## Changing parameters

- Change goals in src/ee4308\_bringup/worlds/world23proj1.sh
- Change other parameters (weight, PID, inflation radius etc) in src/ee4308\_turtle/config/turtle.yaml

## Simulation

Run in gazebo by checking out to the gazebo branch

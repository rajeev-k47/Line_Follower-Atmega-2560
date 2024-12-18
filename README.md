# Line following maze solver robot
This is a Line following robot which optimizes path to solve the maze made using Arduino Mega.

### Components Used :-
1. **Microcontroller** : Arduino Mega
2. **Sensor** : SmartElex RLS-08
3. **Motor** : N20 Gear motor
4. **Motor Driver** : TB6612FNG Dual DC motor driver
5. **Battery** : 11.1v Li-ion battery

### Key Features :-
1. **Algorithm** :- Uses left wall following algorithm to solve the maze.
2. **PID** :- Integrated with PID feedback to prevent the deflection of bot from the line.
3. **End Led** : Turns on a Red LED on dry run and Green LED on active run at the end of maze.
4. **Active run** : Saves the path on the dry run and uses net angle direction algorithm to figure out the shortest path used.

## Watch Video

[![Watch the video](https://img.youtube.com/vi/BMC8-RLoKcI/0.jpg)](https://www.youtube.com/watch?v=BMC8-RLoKcI)

# Dijkstra-Algorithm-for-point-and-rigid-robot

In this project we have implemented Dijkstra Algorithm to find the optimal path between 2 nodes on a given map with certain obstacles
in between like a circle, ellipse, square. rectangle and a polygon. The algorithm has been designed in such a way that the point robot and
rigid robot should not come in contact with the obstacles while exploring the path towards the goal node.


## Steps to run the program

### For running the point robot
```
python3 Dijkstra_point.py
```

### For running the rigid robot

```
python3 Dijkstra_rigid.py
```

After you run the program you will get a prompt wherein you need to input the start coordinates, goal coordinates, and radius and clearance
for the robot.

### Line Equations of Polygon

```
A = (20,120) ; B = (50,150) ; C = (75,120) ; D = (100,150) ; E = (75,185) ; F = (25,185)
AB - Y = X + 100
BC - Y = -1.2X + 210
CD - Y = 1.2X + 30
DE - Y = -1.4X + 290
EF - Y 
FA - Y = 13X - 140

Finding angles between  two intersecting lines to find the new euqtions of lines for rigid robot using :
tan(theta) = (m2 - m1)/(1 + m1m2)
where m1 and m2 are the slopes of the two intersecting lines. 
```
### Line Equations of Rectangle

```
A = (30,68) ; B = (95,30) ; C = (100,39) ; D = (35,76)
AB - Y = -0.58X + 85.4
BC - Y = +1.8X - 141
CD - Y = -0.58X + 96.3
DA - Y = +1.8X + 13

```
### Line Equations of Rhombus

```
A = (200,25) ; B = (225,10) ; C = (250,25) ; D = (225,40)
AB - Y = -0.6X + 145
BC - Y = +0.6X - 125
CD - Y = -0.6X + 175
DA - Y = +0.6X - 95

```

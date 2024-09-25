

<!--  DELETE THE LINES ABOVE THIS AND WRITE YOUR PROJECT README BELOW -->

# Comprobo Warmup Project
*ENGR3590: A Computational Introduction to Robotics*

*Christopher Nie*\
*Charlie Mawn*

The goal of this project is to familiarize students with Ros2, the interface between Python and the Neato\
that we will be using throughout this course. This project first started with simple drive commands using \
the publisher `cmd_vel`, and eventually introduced perception and proportional control. The primary sensors\
used in this project are the built-in lidar and bump sensor. These are the final behaviors programmed into\
our Neato. \

- Teleoperation
- Driving a Square
- Wall Follower
- People Follower 
- Obstacle Avoider
- Finite-State Controller

## Teleoperation

This module allows a person to manually remote control a robot through a wireless connection. Our teleoperation\
catches user input as long as the user is in the terminal that is running the ros2 file. The Neato accepts the\
following input: \

- W: Forward
- A: Rotate Counterclockwise 
- S: Rotate Clockwise
- D: Backward
- X: STOP



## Driving a Square

This module drives the Neato to draw a predefined shape. We chose to draw a square. This module uses an \
open loop controller where the robot performs tasks based on a predefined tasklist, without taking active \
feedback from sensors. We judged that this controller was sufficient for this portion of the project. However,\
This was also our first exposure to the Neato's margin of error. Although it ran successfully in the simulation, \
the real world Neato had to contend with multiple factors such as friction and inertia. Therefore, we had to\
perform some minor tuning to adapt the code to the real world. Although the user was not able to interact much\
with the robot in this module, one direction we could have explored was letting the user decide the shape that \
the Neato would draw.

## Wall Follower

This module was our first exposure to utilizing the lidar sensor. This is one of the more powerful perception tools \
available to the Neato. The Neato returns its Lidar content through the topic `/scan` in a `LaserScan` type message.\ 
`LaserScan.ranges` returns the distances of each degree in an array with length `361`. We store this in a `numpy.array` \
object called `distances`, and create a separate `numpy.array` object called `angles`. Next, we filter out any `distances` \
and `angles` that are greater than `0` and less than `1.7` to reduce noise. 

We then sort the remaining `distances` and `angles` into the **left wall** and **right wall**. We compare the average \ 
distance to both walls. This allows the Neato to determine which wall to follow. We decided that the callback function \
will only manipulate angular velocity to steer the Neato at a constant linear velocity of `0.1`. In order to calculate the angular velocity, the Neato compares the average distance of the wall before it to the average distance of the wall behind \
it. The angular velocity is simply 
```
self.angular_vel = front - back
```
If the average distance of the front is greater, than it will turn counterclockwise. Otherwise, it will turn clockwise.\
We initially had trouble coding this part due to our intuition telling us that "left" was negative and "right" was \
positive. This also swapped the "left wall" and "right wall", which overall meant that this was a debuggin nightmare. \
From this, we took a lesson of testing early and more frequently, as well being more efficient with `print()` statements\
in debugging projects.  

* diagram *

We also decided to add some level of user interaction. The user controls are similar to the `teleop` module.\

- W: Forward (Wall Following ON)
- A: Rotate Counterclockwise (Wall Following OFF)
- S: Rotate clockwise (Wall Following OFF)
- A: Backwards (Wall Following OFF)

This allows the user to manually move the Neato, and allowed us to test the Neato without physically moving the Neato\ However, the user needs to confirm the input with an `enter`. The algorithm that `Wall follower` uses requires \
non-blocking user input. However, we were unable to provide that with our intial `teleop` module. Thus, we used \
multi-threading with the default Python `input()` to achieve a non-blocking variation of `teleop`. 
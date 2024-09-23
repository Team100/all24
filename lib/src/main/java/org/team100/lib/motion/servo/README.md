# servo

A Team 100 "servo" is a composition of

- a _mechanism_
- a _sensor_
- a _controller_

The interfaces distinguish between positional control and velocity control, and also between angular mechanisms, like arms, where the absolute rotational position is what matters, and measurements can be "wrapped" to 360 degrees, and linear mechanisms, like drive wheels or elevators, where the measurement of interest is actually distance.

Some of the implementations use "outboard" control using motor controller logic, but most of them use "onboard" control using logic hosted on the RoboRIO.

These classes are intended to be generally useful, but if one doesn't seem to fit your task, feel free to make more of them, freely cutting and pasting.

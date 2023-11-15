# Layers

Working out the design patterns for pluggable controllers.

The general design has quite a few layers, alternating data and code:

* spec: the thing that tells the robot to do over time.  could be 
a trajectory, or some other form of guidance, like the center point
and radius of an orbit.
* spec follower: reads the spec and the clock and produces references.
* workspace state: describes the robot state in task-appropriate terms, e.g.
cartesian coordinates (and their time-derivatives)
* workspace controller: given references and measurements in workspace,
produces workspace state.
* kinematics transform: translates worksapce to configuration space
* configuration: describes the robot in robot-appropriate terms, e.g. 
joint angles
* configuration controller: given references and measurements in configuration
space, produce actuator state
* actuator state: actuators are velocity closed-loop and arbitrary open-loop
feedforward.
* actuator: executes the velocity goal and applies the feedforward.

So five layers of logic including the kinematics and outboard controller;
there are three layers of controller logic.

Some control systems may obviate or combine some of these, for example
a manual controller could be applied at any level, actuator, configuration,
or workspace.  a trajectory follower may incorporate both the spec follower
and workspace controller in a single unit, like the pure pursuit controller
does.
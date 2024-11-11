# Manual Driving

The DriveManually command uses a selectable control method, using one of these interfaces:

* ModuleStateDriver
* ChassisSpeedDriver
* FieldRelativeDriver

These are used for things like "snaps" where the cartesian component of motion
is human-controlled, but the rotational component is autonomous.

There are several "snap" implementations:

* **ManualWithProfiledHeading:** uses the usual WPI pattern of a timed profile and
  feedback control to stay on the profile.  There's one important subtlety in the
  profile generator: the profile speed and accel are inversely proportional to the
  current robot speed.  The reason is that, when moving near their maximum speeds,
  the drive motors have very little torque available, it's just how motors work.
  If you try to rotate faster, the robot just won't keep up with the profile, it
  will result in controller error and overshoot.  To the driver, the variable profile
  means that the robot won't rotate much when you're driving fast, and it will
  "catch up" when you slow down.

* **ManualWithFullStateHeading:** uses a very simple control law:
  u = K<sub>1</sub> &theta; + K<sub>2</sub> &omega;. There's no profile timing to
  worry about.  The current implementation uses fixed gains, but might be improved
  by gain scheduling according to robot velocity, as above.
  
* **ManualWithMinTimeHeading:** uses the min-time controller, which is a "sliding mode"
  controller that uses a trapezoidal profile.  This driver doesn't yet implement
  variable velocity and so it shouldn't be used.
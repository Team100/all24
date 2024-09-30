# Follower

This package contains trajectory followers, i.e. implementations of DriveMotionController.

These used to be in the "controller" package, which also contains lower-level control logic;
I moved the followers here to draw a sharper distinction.

Trajectory followers produce robot-relative commands, which should be passed directly
to SwerveDriveSubsystem.setChassisSpeedsNormally().
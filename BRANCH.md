This describes the drive_steer_decoupling branch.

In 2024 I thought that the coupling between swerve driving and steering was
the cause of the rotation we're seeing, so I wrote some code to sense
the drive torque and apply opposite torque.

Eventually I tested it, watching the steering while driving around,
and found that, with firm feedback on the steering axes, this effect
doesn't seem to be very large, and the code to counteract it seemed to be
overreacting.

I think that just firming up the steering control is enough of a "fix" for this issue.

When we convert to outboard positional control, the effective "P" value will be astronomical.

I'll remove it from main, to reduce complexity, and I'm leaving it here in a branch,
in case anyone ever wants to pick it up again.

https://docs.google.com/document/d/1Zm6VpteqNMmT0VaTDhN5U6-jF3VS11uCoykzZUIGQdU/edit

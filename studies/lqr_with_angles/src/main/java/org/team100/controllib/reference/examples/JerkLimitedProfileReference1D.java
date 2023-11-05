package org.team100.controllib.reference.examples;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

/**
 * WPI profiles have infinite jerk which makes the feedback controller ring.
 * 
 * This uses the roadrunner jerk-limited profiles and doesn't ring.
 * 
 * Results here:
 * 
 * https://docs.google.com/spreadsheets/d/1miehTmvbdRFs49wy2x8u_3SqVKrNw-oJB5Dv5-uQYo0
 */
public class JerkLimitedProfileReference1D extends Reference1D {
    private static final double maxVel = 0.5;
    private static final double maxAccel = 1.5;
    private static final double maxJerk = 1;

    private final MotionState start;
    private final MotionState goal;
    private final MotionProfile profile;

    public JerkLimitedProfileReference1D() {
        start = new MotionState(0, 0, 0);
        goal = new MotionState(1, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                maxVel,
                maxAccel,
                maxJerk);
    }

    @Override
    public double position(double timeSec) {
        MotionState s = profile.get(timeSec);
        return s.getX();
    }

    @Override
    public double velocity(double timeSec) {
        MotionState s = profile.get(timeSec);
        return s.getV();
    }

    @Override
    public double acceleration(double timeSec) {
        MotionState s = profile.get(timeSec);
        return s.getA();
    }
}

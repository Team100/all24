package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
// import com.acmerobotics.roadrunner.profile.MotionState;
// import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
// import com.acmerobotics.roadrunner.profile.VelocityConstraint;

import org.junit.jupiter.api.Test;

class MotionProfileGeneratorTest {
    private static final double kDelta = 0.001;

    @Test
    void testGenerateSimpleMotionProfile() {
        MotionState start = new MotionState(0,0,0,0);
        MotionState goal = new MotionState(10,0,0,0);
        double maxVel = 1;
        double maxAccel = 1;
        double maxJerk = 1;
        boolean overshoot = false;
        MotionProfile p = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, maxVel, maxAccel, maxJerk, overshoot);

        assertEquals(12, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(0, s0.getV(), kDelta);

        MotionProfile p1 = p.plus(p);
        assertEquals(24, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(0.5, s1.getV(), kDelta);
    }

    @Test
    void testGenerateMotionProfile() {
        MotionState start = new MotionState(0,0,0,0);
        MotionState goal = new MotionState(5,0,0,0);
        VelocityConstraint v = new VelocityConstraint() {

            @Override
            public double get(double s) {
                return 1;
            }

        };
        AccelerationConstraint a = new AccelerationConstraint() {

            @Override
            public double get(double s) {
                return 1;
            }

        };
        double resolution = 1;
        MotionProfile p = MotionProfileGenerator.generateMotionProfile(start, goal, v, a, resolution);

        assertEquals(7, p.getSegments().size());
        assertEquals(0, p.get(0).getX(), kDelta);
        assertEquals(0.5, p.get(1).getX(), kDelta);
        assertEquals(1.5, p.get(2).getX(), kDelta);
        assertEquals(2.5, p.get(3).getX(), kDelta);
        assertEquals(3.5, p.get(4).getX(), kDelta);
        assertEquals(4.5, p.get(5).getX(), kDelta);
        assertEquals(5.0, p.get(6).getX(), kDelta);

        assertEquals(6.0, p.duration(), kDelta);

        MotionState s0 = p.get(0);
        assertEquals(0, s0.getV(), kDelta);

        MotionProfile p1 = p.plus(p);
        assertEquals(12, p1.duration(), kDelta);

        MotionState s1 = p.get(1);
        assertEquals(1, s1.getV(), kDelta);
    }

    @Test
    void testSampleCount() {
        MotionState start = new MotionState(0,0,0,0);
        MotionState goal = new MotionState(5,0,0,0);
        double resolution = 1;
        double length = goal.getX() - start.getX();
        int samples = Math.max(2, (int) Math.ceil(length / resolution));
        assertEquals(5, samples);

    }
    
}

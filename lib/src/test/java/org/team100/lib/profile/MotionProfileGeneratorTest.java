package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
// import com.acmerobotics.roadrunner.profile.MotionState;
// import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
// import com.acmerobotics.roadrunner.profile.VelocityConstraint;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

class MotionProfileGeneratorTest {
    private static final double kDelta = 0.001;

    @Test
    void testGenerateSimpleMotionProfile() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(10, 0, 0, 0);
        double maxVel = 1;
        double maxAccel = 1;
        double maxJerk = 1;
        boolean overshoot = false;
        MotionProfile p = MotionProfileGenerator.generateSimpleMotionProfile(start, goal, maxVel, maxAccel, maxJerk,
                overshoot);

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
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(5, 0, 0, 0);
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
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(5, 0, 0, 0);
        double resolution = 1;
        double length = goal.getX() - start.getX();
        int samples = Math.max(2, (int) Math.ceil(length / resolution));
        assertEquals(5, samples);

    }

    @Test
    void testProfileRestToRest() {
        MotionProfile p;

        // start == end
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0),
                new MotionState(0, 0),
                1, 1, 0, true);
        assertEquals(0, p.duration(), kDelta);

        // 0.1 rad clockwise
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0),
                new MotionState(0.1, 0),
                1, 1, 0, true);
        assertEquals(0.632, p.duration(), kDelta);

        // 0.1 rad counterclockwise
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0),
                new MotionState(-0.1, 0),
                1, 1, 0, true);
        assertEquals(0.632, p.duration(), kDelta);

        // 0.1 rad across zero
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(-0.05, 0),
                new MotionState(0.05, 0),
                1, 1, 0, true);
        assertEquals(0.632, p.duration(), kDelta);

        // 0.1 rad across pi
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(Math.PI - 0.05, 0),
                new MotionState(Math.PI + 0.05, 0),
                1, 1, 0, true);
        assertEquals(0.632, p.duration(), kDelta);
    }

    /** for comparison, the WPI trapezoid does the right thing. */
    @Test
    void testTrapzoid() {
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(1, 1);
        TrapezoidProfile p;

        // no movement -> no duration
        p = new TrapezoidProfile(c,
                new TrapezoidProfile.State(0, 0),
                new TrapezoidProfile.State(0.002, 0));
        // System.out.println(p);
        // assertEquals(0, p.totalTime());

        // no net movement, initial velocity not zero
        p = new TrapezoidProfile(c,
                new TrapezoidProfile.State(0, 0),
                new TrapezoidProfile.State(0.000001, 0.1));
        // System.out.println(p);
        for (double t = 0; t < 0.3; t += 0.02) {
            System.out.printf("%5.3f %5.3f %5.3f\n", t, p.calculate(t).position, p.calculate(t).velocity);
        }
        assertEquals(0.0414, p.totalTime(), kDelta);

    }

    @Test
    void testProfileMovingTowardsGoal() {
        MotionProfile p;

        // start == end with positive velocity
        // if the "violate constraints" flag is false, then this "solves" the problem
        // with negative-infinity acceleration. :-(
        // if the flag is true, then the solution is insane, max accel *away* from the
        // goal
        // try it in reverse

        // p = MotionProfileGenerator.generateSimpleMotionProfile(
        // new MotionState(0, 0.1),
        // new MotionState(0, 0),
        // 1, 1, 1, true);
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0),
                new MotionState(0.001, -0.1),
                1, 1, 1, true);

        System.out.println(p);
        for (double t = 0; t < 1.5; t += 0.05) {
            System.out.printf("%5.3f %s\n", t, p.get(t));
        }
        assertEquals(-1, p.duration(), kDelta);

        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0.1),
                new MotionState(0.1, 0),
                1, 1, 0, true);

        System.out.println(p);
        assertEquals(-1, p.duration(), kDelta);

        // 0.1 rad clockwise
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0.1),
                new MotionState(0.1, 0),
                1, 1, 0, true);

        assertEquals(0.632, p.duration(), kDelta);

        // 0.1 rad counterclockwise
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, -0.1),
                new MotionState(-0.1, 0),
                1, 1, 0, true);

        assertEquals(0.632, p.duration(), kDelta);

        // 0.1 rad across zero
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(-0.05, 0.1),
                new MotionState(0.05, 0),
                1, 1, 0, true);

        assertEquals(0.632, p.duration(), kDelta);

        // 0.1 rad across pi
        p = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(Math.PI - 0.05, 0.1),
                new MotionState(Math.PI + 0.05, 0),
                1, 1, 0, true);
        assertEquals(0.632, p.duration(), kDelta);
    }

    // @Test
    // void testProfileMovingAwayFromGoal() {
    // SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
    // MotionProfile p;
    // // start == end with negative velocity
    // p = ManualWithHeading.updateProfile(speedLimits, new Rotation2d(), -1, new
    // Rotation2d());
    // assertEquals(0, p.duration(), kDelta);

    // // 0.1 rad clockwise
    // p = ManualWithHeading.updateProfile(speedLimits, new Rotation2d(), 0, new
    // Rotation2d(0.1));
    // assertEquals(0.632, p.duration(), kDelta);

    // // 0.1 rad counterclockwise
    // p = ManualWithHeading.updateProfile(speedLimits, new Rotation2d(), 0, new
    // Rotation2d(-0.1));
    // assertEquals(0.632, p.duration(), kDelta);

    // // 0.1 rad across zero
    // p = ManualWithHeading.updateProfile(speedLimits,
    // new Rotation2d(-0.05), 0,
    // new Rotation2d(0.05));
    // assertEquals(0.632, p.duration(), kDelta);

    // // 0.1 rad across pi
    // p = ManualWithHeading.updateProfile(speedLimits,
    // new Rotation2d(Math.PI - 0.05), 0,
    // new Rotation2d(Math.PI + 0.05));
    // assertEquals(0.632, p.duration(), kDelta);
    // }

}

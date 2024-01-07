package org.team100.lib.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class SimulatedHeadingTest {
    private static final double kDelta = 0.001;

    @Test
    void testInitial() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveModuleCollection c = SwerveModuleCollection.get(10, l);
        SimulatedHeading h = new SimulatedHeading(l, c);
        assertEquals(0, h.getHeadingNWU().getRadians(), kDelta);
        assertEquals(0, h.getHeadingRateNWU(), kDelta);
    }

    // TODO: fix this test
    // @Test
    void testTranslation() {
        HAL.initialize(500, 0);
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveModuleCollection c = SwerveModuleCollection.get(10, l);
        SwerveModulePosition[] p = c.positions();
        assertEquals(0, p[0].distanceMeters, kDelta);
        assertEquals(0, p[1].distanceMeters, kDelta);
        assertEquals(0, p[2].distanceMeters, kDelta);
        assertEquals(0, p[3].distanceMeters, kDelta);
        SimulatedHeading h = new SimulatedHeading(l, c);
        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
        // includes discretization
        SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 0, 0.02);
        c.reset();
        // go for 0.4s
        for (int i = 0; i < 20; ++i) {
            c.setDesiredStates(states);
            SimHooks.stepTimingAsync(0.02);
            c.periodic();
        }
        assertEquals(0, h.getHeadingNWU().getRadians(), kDelta);
        assertEquals(0, h.getHeadingRateNWU(), kDelta);
        p = c.positions();
        assertEquals(0.42, p[0].distanceMeters, 0.03);
        assertEquals(0.42, p[1].distanceMeters, 0.03);
        assertEquals(0.42, p[2].distanceMeters, 0.03);
        assertEquals(0.42, p[3].distanceMeters, 0.03);
        HAL.shutdown();
    }

    // TODO: fix this test
    // @Test
    void testRotation() {
        HAL.initialize(500, 0);
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveModuleCollection c = SwerveModuleCollection.get(10, l);
        SimulatedHeading h = new SimulatedHeading(l, c);
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
        // includes discretization
        SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 1, 0.02);

        c.reset();
        // steering velocity is 13 rad/s, we need to go about 2 rad? so wait 0.2 sec?
        for (int i = 0; i < 20; ++i) {
            // get the modules pointing the right way (wait for the steering profiles)
            c.setDesiredStates(states);
            SimHooks.stepTimingAsync(0.02);
            c.periodic();
        }

        // target is 1 rad/sec, we went 0.4 sec. some of that time is spent accelerating
        // the module steering, though the drive motors respond instantly. so this
        // should be something less than 0.4, but it's a little too high?
        assertEquals(0.42, h.getHeadingNWU().getRadians(), 0.03);
        // the rate is what we asked for.
        assertEquals(1, h.getHeadingRateNWU(), kDelta);
        HAL.shutdown();
    }

    // TODO: fix this test
    //@Test
    void testHolonomic() {
        HAL.initialize(500, 0);
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 1);
        // includes discretization
        SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 1, 0.02);
        // these are discretized so not symmetrical
        assertEquals(0.779, states[0].speedMetersPerSecond, kDelta);
        assertEquals(1.268, states[1].speedMetersPerSecond, kDelta);
        assertEquals(0.802, states[2].speedMetersPerSecond, kDelta);
        assertEquals(1.281, states[3].speedMetersPerSecond, kDelta);
        assertEquals(0.279, states[0].angle.getRadians(), kDelta);
        assertEquals(0.170, states[1].angle.getRadians(), kDelta);
        assertEquals(-0.363, states[2].angle.getRadians(), kDelta);
        assertEquals(-0.224, states[3].angle.getRadians(), kDelta);

        SwerveModuleCollection c = SwerveModuleCollection.get(10, l);
        SimulatedHeading h = new SimulatedHeading(l, c);
        c.reset();

        // steering velocity is 13 rad/s, we need to go about 2 rad? so wait 0.2 sec?
        for (int i = 0; i < 20; ++i) {
            // get the modules pointing the right way (wait for the steering profiles)
            c.setDesiredStates(states);
            SimHooks.stepTimingAsync(0.02);
            c.periodic();
        }
        SwerveModuleState[] states2 = c.states();

        // we get back what we put in
        assertEquals(0.779, states2[0].speedMetersPerSecond, kDelta);
        assertEquals(1.268, states2[1].speedMetersPerSecond, kDelta);
        assertEquals(0.802, states2[2].speedMetersPerSecond, kDelta);
        assertEquals(1.281, states2[3].speedMetersPerSecond, kDelta);
        assertEquals(0.279, states2[0].angle.getRadians(), kDelta);
        assertEquals(0.170, states2[1].angle.getRadians(), kDelta);
        assertEquals(-0.363, states2[2].angle.getRadians(), kDelta);
        assertEquals(-0.224, states2[3].angle.getRadians(), kDelta);

        // we wanted to turn 1 rad/s for 0.4s so this is close.
        assertEquals(0.42, h.getHeadingNWU().getRadians(), 0.03);
        // we wanted to move 1 rad/s, so that's what we got.
        assertEquals(1, h.getHeadingRateNWU(), kDelta);
        HAL.shutdown();
    }
}

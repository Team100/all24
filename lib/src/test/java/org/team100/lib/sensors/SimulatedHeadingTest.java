package org.team100.lib.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SimulatedHeadingTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testInitial() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveModuleCollection c = SwerveModuleCollection.get(logger, 10, 20, l);
        SimulatedGyro h = new SimulatedGyro(l, c);
        assertEquals(0, h.getYawNWU().getRadians(), kDelta);
        assertEquals(0, h.getYawRateNWU(), kDelta);
    }

    @Test
    void testTranslation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveModuleCollection c = SwerveModuleCollection.get(logger, 10, 20, l);
        SwerveModulePositions p = c.positions();
        assertEquals(0, p.frontLeft().distanceMeters, kDelta);
        assertEquals(0, p.frontRight().distanceMeters, kDelta);
        assertEquals(0, p.rearLeft().distanceMeters, kDelta);
        assertEquals(0, p.rearRight().distanceMeters, kDelta);
        SimulatedGyro h = new SimulatedGyro(l, c);
        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
        // includes discretization
        SwerveModuleStates states = l.toSwerveModuleStates(speeds, 0);
        c.reset();
        // go for 0.4s
        for (int i = 0; i < 20; ++i) {
            c.setDesiredStates(states);
            stepTime(0.02);
        }
        assertEquals(0, h.getYawNWU().getRadians(), kDelta);
        assertEquals(0, h.getYawRateNWU(), kDelta);
        p = c.positions();
        assertEquals(0.42, p.frontLeft().distanceMeters, 0.03);
        assertEquals(0.42, p.frontRight().distanceMeters, 0.03);
        assertEquals(0.42, p.rearLeft().distanceMeters, 0.03);
        assertEquals(0.42, p.rearRight().distanceMeters, 0.03);
    }

    @Test
    void testRotation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveModuleCollection c = SwerveModuleCollection.get(logger, 10, 20, l);
        SimulatedGyro h = new SimulatedGyro(l, c);
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
        // includes discretization
        SwerveModuleStates states = l.toSwerveModuleStates(speeds, 1);

        c.reset();
        // steering velocity is 13 rad/s, we need to go about 2 rad? so wait 0.2 sec?
        for (int i = 0; i < 20; ++i) {
            // get the modules pointing the right way (wait for the steering profiles)
            c.setDesiredStates(states);
            stepTime(0.02);
        }

        // target is 1 rad/sec, we went 0.4 sec. some of that time is spent accelerating
        // the module steering, though the drive motors respond instantly. so this
        // should be something less than 0.4, but it's a little too high?
        assertEquals(0.42, h.getYawNWU().getRadians(), 0.03);
        // the rate is what we asked for.
        assertEquals(1, h.getYawRateNWU(), kDelta);
    }

    @Test
    void testHolonomic() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 1);
        // includes discretization
        SwerveModuleStates states = l.toSwerveModuleStates(speeds, 1);
        // these are discretized so not symmetrical
        assertEquals(0.779, states.frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(1.268, states.frontRight().speedMetersPerSecond, kDelta);
        assertEquals(0.802, states.rearLeft().speedMetersPerSecond, kDelta);
        assertEquals(1.281, states.rearRight().speedMetersPerSecond, kDelta);
        assertEquals(0.279, states.frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0.170, states.frontRight().angle.get().getRadians(), kDelta);
        assertEquals(-0.363, states.rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(-0.224, states.rearRight().angle.get().getRadians(), kDelta);

        SwerveModuleCollection c = SwerveModuleCollection.get(logger, 10, 20, l);
        SimulatedGyro h = new SimulatedGyro(l, c);
        c.reset();

        // steering velocity is 13 rad/s, we need to go about 2 rad? so wait 0.2 sec?
        for (int i = 0; i < 20; ++i) {
            // get the modules pointing the right way (wait for the steering profiles)
            c.setDesiredStates(states);
            stepTime(0.02);
        }
        SwerveModuleStates states2 = c.states();

        // we get back what we put in
        assertEquals(0.779, states2.frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(1.268, states2.frontRight().speedMetersPerSecond, kDelta);
        assertEquals(0.802, states2.rearLeft().speedMetersPerSecond, kDelta);
        assertEquals(1.281, states2.rearRight().speedMetersPerSecond, kDelta);
        assertEquals(0.279, states2.frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0.170, states2.frontRight().angle.get().getRadians(), kDelta);
        assertEquals(-0.363, states2.rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(-0.224, states2.rearRight().angle.get().getRadians(), kDelta);

        // we wanted to turn 1 rad/s for 0.4s so this is close.
        assertEquals(0.42, h.getYawNWU().getRadians(), 0.03);
        // we wanted to move 1 rad/s, so that's what we got.
        assertEquals(1, h.getYawRateNWU(), kDelta);
    }
}

package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SimpleManualModuleStatesTest {
    private static final double kDelta = 0.001;

    @Test
    void testZero() {
        Supplier<Twist2d> input = () -> new Twist2d();
        SpeedLimits limits = new SpeedLimits(1, 1, 1, 1);
        SimpleManualModuleStates s = new SimpleManualModuleStates(input, limits);
        SwerveModuleState[] ms = s.get();
        assertEquals(0, ms[0].angle.getRadians(), kDelta);
        assertEquals(0, ms[1].angle.getRadians(), kDelta);
        assertEquals(0, ms[2].angle.getRadians(), kDelta);
        assertEquals(0, ms[3].angle.getRadians(), kDelta);

        assertEquals(0, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(0, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(0, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(0, ms[3].speedMetersPerSecond, kDelta);
    }

    @Test
    void testAngle() {
        Supplier<Twist2d> input = () -> new Twist2d(0, 0, 0.5);
        SpeedLimits limits = new SpeedLimits(1, 1, 1, 1);
        SimpleManualModuleStates s = new SimpleManualModuleStates(input, limits);
        SwerveModuleState[] ms = s.get();
        assertEquals(Math.PI / 2, ms[0].angle.getRadians(), kDelta);
        assertEquals(Math.PI / 2, ms[1].angle.getRadians(), kDelta);
        assertEquals(Math.PI / 2, ms[2].angle.getRadians(), kDelta);
        assertEquals(Math.PI / 2, ms[3].angle.getRadians(), kDelta);

        assertEquals(0, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(0, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(0, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(0, ms[3].speedMetersPerSecond, kDelta);
    }

    @Test
    void testDrive() {
        Supplier<Twist2d> input = () -> new Twist2d(0.5, 0, 0);
        SpeedLimits limits = new SpeedLimits(1, 1, 1, 1);
        SimpleManualModuleStates s = new SimpleManualModuleStates(input, limits);
        SwerveModuleState[] ms = s.get();
        assertEquals(0, ms[0].angle.getRadians(), kDelta);
        assertEquals(0, ms[1].angle.getRadians(), kDelta);
        assertEquals(0, ms[2].angle.getRadians(), kDelta);
        assertEquals(0, ms[3].angle.getRadians(), kDelta);

        assertEquals(0.5, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(0.5, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(0.5, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(0.5, ms[3].speedMetersPerSecond, kDelta);
    }

}

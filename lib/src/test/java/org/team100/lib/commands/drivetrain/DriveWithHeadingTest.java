package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.MockHeading;
import org.team100.lib.util.MockTimer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

class DriveWithHeadingTest {
    private static final double kDelta = 0.001;

    // These are members so we can capture them without making them final.
    private Rotation2d desiredRotation = GeometryUtil.kRotationZero;
    private Twist2d desiredTwist = new Twist2d();

    @Test
    void testModeSwitching() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        MockTimer timer = new MockTimer();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                timer,
                rotationSupplier);

        command.initialize();
        command.execute();
        // with a non-null desired rotation we're in snap mode
        assertTrue(command.snapMode);
        desiredRotation = null;
        desiredTwist = new Twist2d(0, 0, 1);
        command.execute();
        // with a nonzero desired twist, we're out of snap mode
        assertFalse(command.snapMode);
        command.end(false);
    }

    @Test
    void testNotSnapMode() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        MockTimer timer = new MockTimer();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                timer,
                rotationSupplier);

        // no desired rotation
        desiredRotation = null;
        desiredTwist = new Twist2d(0, 0, 1);
        command.execute();
        // not in snap mode
        assertFalse(command.snapMode);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        assertEquals(1, robotDrive.twist.dtheta, kDelta);

        desiredTwist = new Twist2d(1, 0, 0);
        command.execute();
        assertFalse(command.snapMode);
        assertEquals(1, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        assertEquals(0, robotDrive.twist.dtheta, kDelta);
        command.end(false);
    }

    @Test
    void testSnapMode() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        MockTimer timer = new MockTimer();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                timer,
                rotationSupplier);

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta
        desiredTwist = new Twist2d(0, 0, 0);
        command.execute();
        // in snap mode
        assertTrue(command.snapMode);
        // there should be a profile
        assertEquals(2.571, command.m_profile.duration(), kDelta);
        // but at t0 it hasn't started yet.
        assertEquals(0, command.m_profile.get(0).getV(), kDelta);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // confirm t=0 implies v=0
        assertEquals(0, robotDrive.twist.dtheta, kDelta);

        // let go of the pov to let the profile run.
        // TODO: if you keep desired rotation the same (not null) it should
        // still execute the profile
        desiredRotation = null;

        timer.time = 1;
        command.execute();
        assertEquals(1, command.m_profile.get(1).getV(), kDelta);
        assertTrue(command.snapMode);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // still pushing since the profile isn't done
        assertEquals(1, robotDrive.twist.dtheta, kDelta);

        // almost done
                timer.time = 2.4;
        command.execute();
        assertEquals(1, command.m_profile.get(1).getV(), kDelta);
        assertTrue(command.snapMode);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // almost done
        assertEquals(0.171, robotDrive.twist.dtheta, kDelta);

        timer.time = 100;
        command.execute();
        assertTrue(command.snapMode);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // there should be no more profile to follow
        assertEquals(0, robotDrive.twist.dtheta, kDelta);

        command.end(false);
    }
}

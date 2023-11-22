package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** Exercise the code. */
class DriveWithHeadingTest {
    Rotation2d desiredRotation = new Rotation2d();
    Twist2d desiredTwist = new Twist2d();

    @Test
    void testSimple() {
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
}

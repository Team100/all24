package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

class DriveRotationTest {
    Fixture fixture = new Fixture();

    double desiredRotation = 1;

    @Test
    void testSimple() {
        SwerveDriveSubsystem robotDrive = fixture.drive;

        Supplier<Double> rot = () -> desiredRotation;

        DriveRotation command = new DriveRotation(
                robotDrive, rot);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        // assertEquals(1, robotDrive.twist.dtheta, 0.001);
        command.end(false);
    }
}

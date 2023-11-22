package org.team100.lib.commands;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Exercise the code. */
class DriveToAprilTagTest {

    @Test
    void testSimple() throws IOException {
        int tagID = 1;
        double xOffset = 0;
        double yOffset = 0;
        MockSwerveDriveSubsystem drivetrain = new MockSwerveDriveSubsystem();
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.1, 0.1),
                new Translation2d(0.1, -0.1),
                new Translation2d(-0.1, 0.1),
                new Translation2d(-0.1, -0.1));
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation
                .blueLayout("2023-chargedup.json");

        DriveToAprilTag command = new DriveToAprilTag(
                tagID,
                xOffset,
                yOffset,
                drivetrain,
                kinematics,
                layout);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }
}

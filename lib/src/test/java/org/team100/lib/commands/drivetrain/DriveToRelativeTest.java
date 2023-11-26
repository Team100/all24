package org.team100.lib.commands.drivetrain;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

class DriveToRelativeTest {

    @Test
    void testSimple() {
        Pose2d relative = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        
        DriveToRelative command = new DriveToRelative(relative, robotDrive);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }
}

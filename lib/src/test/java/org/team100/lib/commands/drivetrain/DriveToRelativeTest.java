package org.team100.lib.commands.drivetrain;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.geometry.Pose2d;

class DriveToRelativeTest {

    @Test
    void testSimple() {
        Pose2d relative = GeometryUtil.kPoseZero;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();

        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);

        HolonomicDriveController3 controller = new HolonomicDriveController3();
        DriveToRelative command = new DriveToRelative(relative, speedLimits, robotDrive, controller);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
    }
}

package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.DriveMotionControllerUtil;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;

class FancyTrajectoryTest extends Fixtured {
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest(logger);
        SwerveDriveSubsystem drive = fixture.drive;
        List<TimingConstraint> constraints = new TimingConstraintFactory(kSmoothKinematicLimits).forTest();
        DriveMotionControllerUtil util = new DriveMotionControllerUtil(logger);
        DriveMotionControllerFactory driveControllerFactory = new DriveMotionControllerFactory(util);
        DrivePIDFController.Log PIDFlog = new DrivePIDFController.Log(logger);
        FancyTrajectory command = new FancyTrajectory(
                logger,
                drive,
                driveControllerFactory.fancyPIDF(PIDFlog),
                constraints);
        command.initialize();
        command.execute();

        assertEquals(0, drive.getState().chassisSpeeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.getState().chassisSpeeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.getState().chassisSpeeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

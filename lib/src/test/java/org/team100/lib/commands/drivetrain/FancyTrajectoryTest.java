package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.follower.DrivePIDFFollower;
import org.team100.lib.follower.DriveTrajectoryFollowerFactory;
import org.team100.lib.follower.DriveTrajectoryFollowerUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestSupplierLogger;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;

class FancyTrajectoryTest extends Fixtured {
    private static final LoggerFactory logger = new TestSupplierLogger(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();
        SwerveDriveSubsystem drive = fixture.drive;
        List<TimingConstraint> constraints = new TimingConstraintFactory(kSmoothKinematicLimits).forTest();
        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DriveTrajectoryFollowerFactory driveControllerFactory = new DriveTrajectoryFollowerFactory(util);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(logger);
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

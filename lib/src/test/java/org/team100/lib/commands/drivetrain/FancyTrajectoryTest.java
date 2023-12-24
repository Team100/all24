package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

class FancyTrajectoryTest {

    @Test
    void testSimple() {
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.2, 0.2);
        SwerveKinematicLimits kSmoothKinematicLimits = new SwerveKinematicLimits(4.5, 4.4, 4.4, 13, 7);

        MockSwerveDriveSubsystem drive = new MockSwerveDriveSubsystem();
        TrajectoryPlanner planner = new TrajectoryPlanner(kinematics, kSmoothKinematicLimits);
        FancyTrajectory command = new FancyTrajectory(
                kinematics,
                kSmoothKinematicLimits,
                drive,
                planner);

        command.initialize();
        command.execute();

        assertEquals(0, drive.speeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

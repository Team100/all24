package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.TrajectoryPlanner;

class FancyTrajectoryTest {

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();
        MockSwerveDriveSubsystem drive = new MockSwerveDriveSubsystem();
        TrajectoryPlanner planner = new TrajectoryPlanner(kSmoothKinematicLimits);
        FancyTrajectory command = new FancyTrajectory(drive, planner);

        command.initialize();
        command.execute();

        assertEquals(0, drive.speeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

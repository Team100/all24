package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.TrajectoryPlanner;

class FancyTrajectoryTest extends Fixtured {

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();
        SwerveDriveSubsystem drive = fixture.drive;
        TrajectoryPlanner planner = new TrajectoryPlanner(kSmoothKinematicLimits);
        FancyTrajectory command = new FancyTrajectory(drive, planner, kSmoothKinematicLimits);

        command.initialize();
        command.execute();

        assertEquals(0, drive.speeds(0.02).vxMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds(0.02).vyMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds(0.02).omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

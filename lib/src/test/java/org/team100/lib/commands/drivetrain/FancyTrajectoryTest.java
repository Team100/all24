package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;

class FancyTrajectoryTest extends Fixtured {
    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();
        SwerveDriveSubsystem drive = fixture.drive;
        List<TimingConstraint> constraints = new TimingConstraintFactory(kSmoothKinematicLimits).forTest();

        FancyTrajectory command = new FancyTrajectory(drive, constraints);
        FancyTrajectory.shutDownForTest();
        command.initialize();
        command.execute100(0.02);

        assertEquals(0, drive.speeds(0.02).vxMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds(0.02).vyMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds(0.02).omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

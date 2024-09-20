package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;

class FancyTrajectoryTest extends Fixtured {
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    @Test
    void testSimple() {
        SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest(logger);
        SwerveDriveSubsystem drive = fixture.drive;
        List<TimingConstraint> constraints = new TimingConstraintFactory(kSmoothKinematicLimits).forTest();

        FancyTrajectory command = new FancyTrajectory(logger, drive, constraints);
        FancyTrajectory.shutDownForTest();
        command.initialize();
        command.execute100(0.02);

        assertEquals(0, drive.getState().chassisSpeeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.getState().chassisSpeeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.getState().chassisSpeeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

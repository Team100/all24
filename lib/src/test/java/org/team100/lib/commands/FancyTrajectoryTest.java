package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

class FancyTrajectoryTest {

    @Test
    void testSimple() {
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.1, 0.1),
                new Translation2d(0.1, -0.1),
                new Translation2d(-0.1, 0.1),
                new Translation2d(-0.1, -0.1));

        SwerveKinematicLimits kSmoothKinematicLimits = new SwerveKinematicLimits();
        kSmoothKinematicLimits.kMaxDriveVelocity = 5.05 * .9;
        kSmoothKinematicLimits.kMaxDriveAcceleration = 4.4;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);

        MockSwerveDriveSubsystem drive = new MockSwerveDriveSubsystem();
        FancyTrajectory command = new FancyTrajectory(
                kinematics,
                kSmoothKinematicLimits,
                drive);

        command.initialize();
        command.execute();

        assertEquals(0, drive.speeds().vxMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds().vyMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds().omegaRadiansPerSecond, 0.001);

        command.end(false);
    }
}

package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class FancyTrajectoryTest {

    private static final double kMaxVelocityMetersPerSecond = 5.05; // Calibrated 3/12 on Comp Bot
    private static final double kMaxAccelerationMetersPerSecondSquared = 4.4;

    private static final double kDriveTrackwidthMeters = 0.52705; // DONE Measure and set trackwidth
    private static final double kDriveWheelbaseMeters = 0.52705; // DONE Measure and set wheelbase

    private static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(kDriveTrackwidthMeters / 2.0, -kDriveWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-kDriveTrackwidthMeters / 2.0, -kDriveWheelbaseMeters / 2.0));

    private static final SwerveKinematicLimits kSmoothKinematicLimits = new SwerveKinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond * .9;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kMaxAccelerationMetersPerSecondSquared;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    class MockSwerveDriveSubsystem implements SwerveDriveSubsystemInterface {

        Pose2d pose = new Pose2d();
        double output = 0;
        boolean stopped = false;
        ChassisSpeeds speeds = null;

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public void stop() {
            stopped = true;
        }

        @Override
        public SwerveDriveSubsystem get() {
            return null;
        }

        @Override
        public ChassisSpeeds speeds() {
            return speeds;
        }

        @Override
        public void driveInFieldCoords(Twist2d twist) {
            output = twist.dtheta;
        }

        @Override
        public void setChassisSpeeds(ChassisSpeeds speeds) {
            output = speeds.omegaRadiansPerSecond;
        }

        @Override
        public void setRawModuleStates(SwerveModuleState[] states) {
        }
    }

    // this crashes but i don't know why
    // @Test
    void testTiming() {
        double startTime = Timer.getFPGATimestamp();
        // this changes the fpga timestamp used in testing.
        SimHooks.stepTiming(1);
        double endTime = Timer.getFPGATimestamp();
        assertEquals(1, endTime - startTime, 0.1);
    }

    // TODO fix this
    //@Test
    void testSimple() {
        MockSwerveDriveSubsystem drive = new MockSwerveDriveSubsystem();
        FancyTrajectory fancy = new FancyTrajectory(kKinematics, kSmoothKinematicLimits, drive);
        fancy.initialize();
        fancy.execute();
        assertEquals(0, drive.speeds.vxMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds.vyMetersPerSecond, 0.001);
        assertEquals(0, drive.speeds.omegaRadiansPerSecond, 0.001);
    }

}

package org.team100.lib.motion.drivetrain;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.MockExperiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveDriveSubsystemTest {
    @Test
    void testSimple() {
        MockHeading heading = new MockHeading();
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.1, 0.1),
                new Translation2d(0.1, -0.1),
                new Translation2d(-0.1, 0.1),
                new Translation2d(-0.1, -0.1));
        Rotation2d gyroAngle = GeometryUtil.kRotationZero;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Pose2d initialPoseMeters = GeometryUtil.kPoseZero;

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroAngle, modulePositions, initialPoseMeters);

        FrameTransform frameTransform = new FrameTransform();
        MockExperiments experiments = new MockExperiments();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        SwerveModuleCollectionInterface modules = new NullSwerveModuleCollection();

        SwerveLocal swerveLocal = new SwerveLocal(experiments, speedLimits, kinematics, modules);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                heading,
                poseEstimator,
                frameTransform,
                swerveLocal,
                () -> DriverControl.Speed.NORMAL);
        // try all the actuators
        drive.periodic();
        drive.driveInFieldCoords(new Twist2d(1, 1, 1));

        drive.periodic();
        drive.setChassisSpeeds(new ChassisSpeeds());

        drive.periodic();
        drive.setRawModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        });

        drive.periodic();
        drive.defense();

        drive.periodic();
        drive.stop();

        drive.close();
    }
}

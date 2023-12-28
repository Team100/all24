package org.team100.lib.motion.drivetrain;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveDriveSubsystemTest {
    @Test
    void testSimple() {
        Fixture fixture = new Fixture();

        MockHeading heading = new MockHeading();
        
        Rotation2d gyroAngle = GeometryUtil.kRotationZero;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Pose2d initialPoseMeters = GeometryUtil.kPoseZero;

        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinodynamics.getKinematics(), gyroAngle, modulePositions, initialPoseMeters);

        FrameTransform frameTransform = new FrameTransform();
        SwerveModuleCollection modules = fixture.collection;

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, modules);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                heading,
                poseEstimator,
                frameTransform,
                swerveLocal,
                () -> DriverControl.Speed.NORMAL);
        // try all the actuators
        drive.periodic();
        drive.driveInFieldCoords(new Twist2d(1, 1, 1), 0.02);

        drive.periodic();
        drive.setChassisSpeeds(new ChassisSpeeds(), 0.02);

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

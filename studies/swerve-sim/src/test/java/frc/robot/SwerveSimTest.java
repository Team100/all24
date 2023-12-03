package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import util.PinkNoise;

public class SwerveSimTest {
    private static final double DELTA = 0.001;

    Drivetrain newDrivetrain() {
        DriveControllers controllers = new DriveControllersFactory().get(Identity.BLANK);
        HolonomicDriveController3 controller = new HolonomicDriveController3(controllers);
        AnalogGyro gyro = new AnalogGyro(0);
        VeeringCorrection veering = new VeeringCorrection(() -> -1.0 * gyro.getRate());
        FrameTransform m_frameTransform = new FrameTransform(veering);
        Drivetrain m_swerve = new Drivetrain(
                gyro,
                () -> new PinkNoise.None(),
                controller,
                m_frameTransform);
        m_swerve.simulationInit();
        m_swerve.m_frontLeft.m_drivePIDController.reset();
        m_swerve.m_frontRight.m_drivePIDController.reset();
        m_swerve.m_backLeft.m_drivePIDController.reset();
        m_swerve.m_backRight.m_drivePIDController.reset();

        // weird that you have to do this; a simulation artifact?
        m_swerve.m_frontLeft.m_driveEncoder.reset();
        m_swerve.m_frontRight.m_driveEncoder.reset();
        m_swerve.m_backLeft.m_driveEncoder.reset();
        m_swerve.m_backRight.m_driveEncoder.reset();
        m_swerve.m_gyro.reset();

        // reset odometry *after* all the components are reset.
        m_swerve.resetOdometry(new Pose2d());
        return m_swerve;
    }

    /** verify that not moving has no effect */
    @Test
    void noopTest() {
        assertTrue(HAL.initialize(500, 0));
        Drivetrain m_swerve = newDrivetrain();
        try {
            final Pose2d initialPose = m_swerve.getPose();
            assertEquals(0, initialPose.getX(), DELTA, "initial x");
            assertEquals(0, initialPose.getY(), DELTA, "initial y");
            assertEquals(0, initialPose.getRotation().getRadians(), DELTA, "initial rot");
            m_swerve.setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0))
            });
            m_swerve.simulationPeriodic();
            final Pose2d finalPose = m_swerve.getPose();
            assertEquals(0, finalPose.getX(), DELTA, "final x");
            assertEquals(0, finalPose.getY(), DELTA, "final y");
            assertEquals(0, finalPose.getRotation().getRadians(), DELTA, "final rot");
        } finally {
            m_swerve.close(); // release the HAL stuff
        }
    }

    /** apply x velocity, see x displacement */
    @Test
    void translationTest() {
        assertTrue(HAL.initialize(500, 0));
        Drivetrain m_swerve = newDrivetrain();
        try {
            final Pose2d initialPose = m_swerve.getPose();
            // at the origin
            assertEquals(0, initialPose.getX(), DELTA, "initial x");
            assertEquals(0, initialPose.getY(), DELTA, "initial y");
            assertEquals(0, initialPose.getRotation().getRadians(), DELTA, "initial rot");

            // no motors running
            assertEquals(0, m_swerve.m_frontLeft.getDriveOutput(), DELTA, "FL output");
            assertEquals(0, m_swerve.m_frontRight.getDriveOutput(), DELTA, "FR output");
            assertEquals(0, m_swerve.m_backLeft.getDriveOutput(), DELTA, "BL output");
            assertEquals(0, m_swerve.m_backRight.getDriveOutput(), DELTA, "BR output");

            // new desired state is 1 m/s
            m_swerve.setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0))
            });

            // controller setpoints should now be 1 m/s
            assertEquals(1, m_swerve.m_frontLeft.m_drivePIDController.getSetpoint(), DELTA, "FL setpoint");
            assertEquals(1, m_swerve.m_frontRight.m_drivePIDController.getSetpoint(), DELTA, "FR setpoint");
            assertEquals(1, m_swerve.m_backLeft.m_drivePIDController.getSetpoint(), DELTA, "BL setpoint");
            assertEquals(1, m_swerve.m_backRight.m_drivePIDController.getSetpoint(), DELTA, "BR setpoint");

            // controller errors should now also be 1 m/s
            assertEquals(1, m_swerve.m_frontLeft.m_drivePIDController.getPositionError(), DELTA, "FL setpoint");
            assertEquals(1, m_swerve.m_frontRight.m_drivePIDController.getPositionError(), DELTA, "FR setpoint");
            assertEquals(1, m_swerve.m_backLeft.m_drivePIDController.getPositionError(), DELTA, "BL setpoint");
            assertEquals(1, m_swerve.m_backRight.m_drivePIDController.getPositionError(), DELTA, "BR setpoint");

            // so since the error is 1 and the PID are (0.1,0,0) controller output should be
            // 0.1
            assertEquals(0.1, m_swerve.m_frontLeft.driveOutput, DELTA, "FL ctrl output");
            assertEquals(0.1, m_swerve.m_frontRight.driveOutput, DELTA, "FR ctrl output");
            assertEquals(0.1, m_swerve.m_backLeft.driveOutput, DELTA, "BL ctrl output");
            assertEquals(0.1, m_swerve.m_backRight.driveOutput, DELTA, "BR ctrl output");

            // kv 0.15 ks 0.001, v = 1, so ff is 0.151
            assertEquals(0.151, m_swerve.m_frontLeft.driveFeedforward, DELTA, "FL ff");
            assertEquals(0.151, m_swerve.m_frontRight.driveFeedforward, DELTA, "FR ff");
            assertEquals(0.151, m_swerve.m_backLeft.driveFeedforward, DELTA, "BL ff");
            assertEquals(0.151, m_swerve.m_backRight.driveFeedforward, DELTA, "BR ff");

            // add ctrl and ff, 0.251
            assertEquals(0.251, m_swerve.m_frontLeft.getDriveOutput(), DELTA, "FL output");
            assertEquals(0.251, m_swerve.m_frontRight.getDriveOutput(), DELTA, "FR output");
            assertEquals(0.251, m_swerve.m_backLeft.getDriveOutput(), DELTA, "BL output");
            assertEquals(0.251, m_swerve.m_backRight.getDriveOutput(), DELTA, "BR output");
            m_swerve.simulationPeriodic(0.02);
            // m_swerve.updateOdometry();
            final Pose2d finalPose = m_swerve.getPose();

            // since the feedforward is treated as correct but the controller
            // is boosting then this overshoots. which is fine? i guess?
            assertEquals(0.025, finalPose.getX(), DELTA, "final x");
            assertEquals(0, finalPose.getY(), DELTA, "final y");
            assertEquals(0, finalPose.getRotation().getRadians(), DELTA, "final rot");
        } finally {
            m_swerve.close(); // release the HAL stuff
        }
    }

    /**
     * steer and drive to spin the robot
     */
    @Test
    void rotationTest() {
        assertTrue(HAL.initialize(500, 0));
        Drivetrain m_swerve = newDrivetrain();
        try {
            final Pose2d initialPose = m_swerve.getPose();
            // at the origin
            assertEquals(0, initialPose.getX(), DELTA, "initial x");
            assertEquals(0, initialPose.getY(), DELTA, "initial y");
            assertEquals(0, initialPose.getRotation().getRadians(), DELTA, "initial rot");

            // no motors running
            assertEquals(0, m_swerve.m_frontLeft.getDriveOutput(), DELTA, "FL output");
            assertEquals(0, m_swerve.m_frontRight.getDriveOutput(), DELTA, "FR output");
            assertEquals(0, m_swerve.m_backLeft.getDriveOutput(), DELTA, "BL output");
            assertEquals(0, m_swerve.m_backRight.getDriveOutput(), DELTA, "BR output");

            // so i can just force the simulated encoders to the settings i want.
            m_swerve.m_frontLeft.m_TurnEncoderSim.setDistance(-Math.PI / 4);
            m_swerve.m_frontRight.m_TurnEncoderSim.setDistance(Math.PI / 4);
            m_swerve.m_backLeft.m_TurnEncoderSim.setDistance(Math.PI / 4);
            m_swerve.m_backRight.m_TurnEncoderSim.setDistance(-Math.PI / 4);

            // verify those settings.
            assertEquals(-0.785, m_swerve.m_frontLeft.getPosition().angle.getRadians(), DELTA, "FL angle");
            assertEquals(0.785, m_swerve.m_frontRight.getPosition().angle.getRadians(), DELTA, "FR angle");
            assertEquals(0.785, m_swerve.m_backLeft.getPosition().angle.getRadians(), DELTA, "BL angle");
            assertEquals(-0.785, m_swerve.m_backRight.getPosition().angle.getRadians(), DELTA, "BR angle");

            // has not moved yet
            assertEquals(0, m_swerve.m_frontLeft.getPosition().distanceMeters, DELTA, "FL m");
            assertEquals(0, m_swerve.m_frontRight.getPosition().distanceMeters, DELTA, "FR m");
            assertEquals(0, m_swerve.m_backLeft.getPosition().distanceMeters, DELTA, "BL m");
            assertEquals(0, m_swerve.m_backRight.getPosition().distanceMeters, DELTA, "BR m");

            // spin positive = CCW, right forward left backward.
            m_swerve.setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(-1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(-1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0))
            });

            // controller setpoints should now be +/-1 m/s
            assertEquals(-1, m_swerve.m_frontLeft.m_drivePIDController.getSetpoint(), DELTA, "FL setpoint");
            assertEquals(1, m_swerve.m_frontRight.m_drivePIDController.getSetpoint(), DELTA, "FR setpoint");
            assertEquals(-1, m_swerve.m_backLeft.m_drivePIDController.getSetpoint(), DELTA, "BL setpoint");
            assertEquals(1, m_swerve.m_backRight.m_drivePIDController.getSetpoint(), DELTA, "BR setpoint");

            // controller errors should now also be 1 m/s
            assertEquals(-1, m_swerve.m_frontLeft.m_drivePIDController.getPositionError(), DELTA, "FL setpoint");
            assertEquals(1, m_swerve.m_frontRight.m_drivePIDController.getPositionError(), DELTA, "FR setpoint");
            assertEquals(-1, m_swerve.m_backLeft.m_drivePIDController.getPositionError(), DELTA, "BL setpoint");
            assertEquals(1, m_swerve.m_backRight.m_drivePIDController.getPositionError(), DELTA, "BR setpoint");

            // so since the error is 1 and the PID are (0.1,0,0) controller output should be
            // 0.1
            assertEquals(-0.1, m_swerve.m_frontLeft.driveOutput, DELTA, "FL ctrl output");
            assertEquals(0.1, m_swerve.m_frontRight.driveOutput, DELTA, "FR ctrl output");
            assertEquals(-0.1, m_swerve.m_backLeft.driveOutput, DELTA, "BL ctrl output");
            assertEquals(0.1, m_swerve.m_backRight.driveOutput, DELTA, "BR ctrl output");

            // kv 0.15 ks 0.001, v = 1, so ff is 0.151
            assertEquals(-0.151, m_swerve.m_frontLeft.driveFeedforward, DELTA, "FL ff");
            assertEquals(0.151, m_swerve.m_frontRight.driveFeedforward, DELTA, "FR ff");
            assertEquals(-0.151, m_swerve.m_backLeft.driveFeedforward, DELTA, "BL ff");
            assertEquals(0.151, m_swerve.m_backRight.driveFeedforward, DELTA, "BR ff");

            // add ctrl and ff, 0.251. 0.25, maybe this is a rounding issue?
            assertEquals(-0.25, m_swerve.m_frontLeft.getDriveOutput(), DELTA, "FL output");
            assertEquals(0.251, m_swerve.m_frontRight.getDriveOutput(), DELTA, "FR output");
            assertEquals(-0.25, m_swerve.m_backLeft.getDriveOutput(), DELTA, "BL output");
            assertEquals(0.251, m_swerve.m_backRight.getDriveOutput(), DELTA, "BR output");
            m_swerve.simulationPeriodic(0.02);
            m_swerve.updateOdometry();

            // each wheel should have moved the same as the displacement case above
            assertEquals(-0.025, m_swerve.m_frontLeft.getPosition().distanceMeters, DELTA, "FL m");
            assertEquals(0.025, m_swerve.m_frontRight.getPosition().distanceMeters, DELTA, "FR m");
            assertEquals(-0.025, m_swerve.m_backLeft.getPosition().distanceMeters, DELTA, "BL m");
            assertEquals(0.025, m_swerve.m_backRight.getPosition().distanceMeters, DELTA, "BR m");

            // no x or y movement. weird that it's not *quite* zero.
            assertEquals(0.001, m_swerve.speeds.vxMetersPerSecond, DELTA, "chassis speed x");
            assertEquals(0, m_swerve.speeds.vyMetersPerSecond, DELTA, "chassis speed y");

            // this is NWU, CCW+, so should be positive.
            assertEquals(2.313, m_swerve.speeds.omegaRadiansPerSecond, DELTA, "chassis speed omega");

            // look at the pose we're maintaining
            assertEquals(0, m_swerve.getPose().getX(), DELTA, "pose x");
            assertEquals(0, m_swerve.getPose().getY(), DELTA, "pose y");
            assertEquals(0.046, m_swerve.getPose().getRotation().getRadians(), DELTA, "pose rot");

            // look at the gyro, note NWU/NED difference, also one is rad the other deg
            assertEquals(0.046, m_swerve.m_gyro.getRotation2d().getRadians(), DELTA, "gyro rotation");
            assertEquals(-2.65, m_swerve.m_gyro.getAngle(), DELTA, "gyro angle");

            final Pose2d finalPose = m_swerve.getPose();

            assertEquals(0, finalPose.getX(), DELTA, "estimate x");
            assertEquals(0, finalPose.getY(), DELTA, "estimate y");
            assertEquals(0.046, finalPose.getRotation().getRadians(), DELTA, "estimate rot");
        } finally {
            m_swerve.close(); // release the HAL stuff
        }
    }

    /**
     * rotate the robot and then translate it.
     */
    @Test
    void rotatedTranslationTest() {
        assertTrue(HAL.initialize(500, 0));
        Drivetrain m_swerve = newDrivetrain();
        try {
            final Pose2d initialPose = m_swerve.getPose();
            // at the origin
            assertEquals(0, initialPose.getX(), DELTA, "initial x");
            assertEquals(0, initialPose.getY(), DELTA, "initial y");
            assertEquals(0, initialPose.getRotation().getRadians(), DELTA, "initial rot");

            // no motors running
            assertEquals(0, m_swerve.m_frontLeft.getDriveOutput(), DELTA, "FL output");
            assertEquals(0, m_swerve.m_frontRight.getDriveOutput(), DELTA, "FR output");
            assertEquals(0, m_swerve.m_backLeft.getDriveOutput(), DELTA, "BL output");
            assertEquals(0, m_swerve.m_backRight.getDriveOutput(), DELTA, "BR output");

            // wheels pointing ahead.
            assertEquals(0, m_swerve.m_frontLeft.getPosition().angle.getRadians(), DELTA, "FL angle");
            assertEquals(0, m_swerve.m_frontRight.getPosition().angle.getRadians(), DELTA, "FR angle");
            assertEquals(0, m_swerve.m_backLeft.getPosition().angle.getRadians(), DELTA, "BL angle");
            assertEquals(0, m_swerve.m_backRight.getPosition().angle.getRadians(), DELTA, "BR angle");

            // has not moved yet
            assertEquals(0, m_swerve.m_frontLeft.getPosition().distanceMeters, DELTA, "FL m");
            assertEquals(0, m_swerve.m_frontRight.getPosition().distanceMeters, DELTA, "FR m");
            assertEquals(0, m_swerve.m_backLeft.getPosition().distanceMeters, DELTA, "BL m");
            assertEquals(0, m_swerve.m_backRight.getPosition().distanceMeters, DELTA, "BR m");

            // pose is zero
            assertEquals(0, m_swerve.getPose().getX(), DELTA, "pose x");
            assertEquals(0, m_swerve.getPose().getY(), DELTA, "pose y");
            assertEquals(0, m_swerve.getPose().getRotation().getRadians(), DELTA, "pose rot");

            // gyro is zero
            assertEquals(0, m_swerve.m_gyro.getRotation2d().getRadians(), DELTA, "gyro rotation");
            assertEquals(0, m_swerve.m_gyro.getAngle(), DELTA, "gyro angle");

            // force gyro to angled position, +pi/2 in NWU, so -90 deg in NED
            m_swerve.gyroSim.setAngle(-90);

            // need to reset the odometry with the actual robot pose since we rotated it.
            m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.m_gyro.getRotation2d())); // pi/2

            // pose should be rotated.
            assertEquals(0, m_swerve.getPose().getX(), DELTA, "pose x");
            assertEquals(0, m_swerve.getPose().getY(), DELTA, "pose y");
            assertEquals(Math.PI / 2, m_swerve.getPose().getRotation().getRadians(), DELTA, "pose rot");

            // verify that the embedded odometry is doing the right thing
            SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_swerve.m_kinematics,
                    new Rotation2d(Math.PI / 2), new SwerveModulePosition[] {
                            m_swerve.m_frontLeft.getPosition(),
                            m_swerve.m_frontRight.getPosition(),
                            m_swerve.m_backLeft.getPosition(),
                            m_swerve.m_backRight.getPosition()
                    }, m_swerve.getPose());

            Pose2d odoPose = m_odometry.getPoseMeters();
            assertEquals(0, odoPose.getX(), DELTA, "x odo pose");
            assertEquals(0, odoPose.getY(), DELTA, "y odo pose");
            assertEquals(Math.PI / 2, odoPose.getRotation().getRadians(), DELTA, "theta odo pose");

            // verify placement
            assertEquals(Math.PI / 2, m_swerve.m_gyro.getRotation2d().getRadians(), DELTA, "gyro rotation");
            assertEquals(-90, m_swerve.m_gyro.getAngle(), DELTA, "gyro angle");

            // drive ahead, as above case, should be +y direction.
            m_swerve.setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0)),
                    new SwerveModuleState(1, new Rotation2d(0))
            });
            // controller setpoints should now be +/-1 m/s
            assertEquals(1, m_swerve.m_frontLeft.m_drivePIDController.getSetpoint(), DELTA, "FL setpoint");
            assertEquals(1, m_swerve.m_frontRight.m_drivePIDController.getSetpoint(), DELTA, "FR setpoint");
            assertEquals(1, m_swerve.m_backLeft.m_drivePIDController.getSetpoint(), DELTA, "BL setpoint");
            assertEquals(1, m_swerve.m_backRight.m_drivePIDController.getSetpoint(), DELTA, "BR setpoint");

            // controller errors should now also be 1 m/s
            assertEquals(1, m_swerve.m_frontLeft.m_drivePIDController.getPositionError(), DELTA, "FL setpoint");
            assertEquals(1, m_swerve.m_frontRight.m_drivePIDController.getPositionError(), DELTA, "FR setpoint");
            assertEquals(1, m_swerve.m_backLeft.m_drivePIDController.getPositionError(), DELTA, "BL setpoint");
            assertEquals(1, m_swerve.m_backRight.m_drivePIDController.getPositionError(), DELTA, "BR setpoint");

            // so since the error is 1 and the PID are (0.1,0,0) controller output should be
            // 0.1
            assertEquals(0.1, m_swerve.m_frontLeft.driveOutput, DELTA, "FL ctrl output");
            assertEquals(0.1, m_swerve.m_frontRight.driveOutput, DELTA, "FR ctrl output");
            assertEquals(0.1, m_swerve.m_backLeft.driveOutput, DELTA, "BL ctrl output");
            assertEquals(0.1, m_swerve.m_backRight.driveOutput, DELTA, "BR ctrl output");

            // kv 0.15 ks 0.001, v = 1, so ff is 0.151
            assertEquals(0.151, m_swerve.m_frontLeft.driveFeedforward, DELTA, "FL ff");
            assertEquals(0.151, m_swerve.m_frontRight.driveFeedforward, DELTA, "FR ff");
            assertEquals(0.151, m_swerve.m_backLeft.driveFeedforward, DELTA, "BL ff");
            assertEquals(0.151, m_swerve.m_backRight.driveFeedforward, DELTA, "BR ff");

            // add ctrl and ff, 0.251
            assertEquals(0.251, m_swerve.m_frontLeft.getDriveOutput(), DELTA, "FL output");
            assertEquals(0.251, m_swerve.m_frontRight.getDriveOutput(), DELTA, "FR output");
            assertEquals(0.251, m_swerve.m_backLeft.getDriveOutput(), DELTA, "BL output");
            assertEquals(0.251, m_swerve.m_backRight.getDriveOutput(), DELTA, "BR output");

            // no steering output
            assertEquals(0, m_swerve.m_frontLeft.getTurnOutput(), DELTA, "FL turn utput");
            assertEquals(0, m_swerve.m_frontRight.getTurnOutput(), DELTA, "FR turn output");
            assertEquals(0, m_swerve.m_backLeft.getTurnOutput(), DELTA, "BL turn output");
            assertEquals(0, m_swerve.m_backRight.getTurnOutput(), DELTA, "BR turn output");
            m_swerve.simulationPeriodic(0.02);
            m_swerve.updateOdometry();

            // each wheel should have moved the same as the displacement case above
            assertEquals(0.025, m_swerve.m_frontLeft.getPosition().distanceMeters, DELTA, "FL m");
            assertEquals(0.025, m_swerve.m_frontRight.getPosition().distanceMeters, DELTA, "FR m");
            assertEquals(0.025, m_swerve.m_backLeft.getPosition().distanceMeters, DELTA, "BL m");
            assertEquals(0.025, m_swerve.m_backRight.getPosition().distanceMeters, DELTA, "BR m");

            m_odometry.update(m_swerve.m_gyro.getRotation2d(), new SwerveModulePosition[] {
                    m_swerve.m_frontLeft.getPosition(),
                    m_swerve.m_frontRight.getPosition(),
                    m_swerve.m_backLeft.getPosition(),
                    m_swerve.m_backRight.getPosition()
            });

            odoPose = m_odometry.getPoseMeters();

            assertEquals(0, odoPose.getX(), DELTA, "x odo pose");
            assertEquals(0.025, odoPose.getY(), DELTA, "y odo pose");
            assertEquals(Math.PI / 2, odoPose.getRotation().getRadians(), DELTA, "theta odo pose");

            // chassis moving in x (ahead), note this isn't field relative, it's chassis
            // relative.
            assertEquals(1.25, m_swerve.speeds.vxMetersPerSecond, DELTA, "chassis speed x");
            assertEquals(0, m_swerve.speeds.vyMetersPerSecond, DELTA, "chassis speed y");
            assertEquals(0, m_swerve.speeds.omegaRadiansPerSecond, DELTA, "chassis speed omega");

            // pose shows movement in y, also remember rotation PI/2
            assertEquals(0, m_swerve.getPose().getX(), DELTA, "pose x");
            assertEquals(0.025, m_swerve.getPose().getY(), DELTA, "pose y"); // <= broken?
            assertEquals(Math.PI / 2, m_swerve.getPose().getRotation().getRadians(), DELTA, "pose rot");

            // same rotation
            assertEquals(Math.PI / 2, m_swerve.m_gyro.getRotation2d().getRadians(), DELTA, "gyro rotation");
            assertEquals(-90, m_swerve.m_gyro.getAngle(), DELTA, "gyro angle");

            final Pose2d finalPose = m_swerve.getPose();

            assertEquals(0, finalPose.getX(), DELTA, "estimate x");
            assertEquals(0.025, finalPose.getY(), DELTA, "estimate y"); // <= broken?
            assertEquals(Math.PI / 2, finalPose.getRotation().getRadians(), DELTA, "estimate rot");
        } finally {
            m_swerve.close(); // release the HAL stuff
        }
    }
}

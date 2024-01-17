package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.drive.FalconDriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.SelectableVelocityServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * For outboard closed-loop control.
 */
public class AMCANSwerveModule100 extends SwerveModule100 {
    // AndyMark Swerve & Steer has 4 inch wheel
    private static final double kWheelDiameterM = 0.1016;
    // see andymark.com/products/swerve-and-steer
    // the true value is 6.67 but the measured value is 90% of that,
    // maybe because the wheel measurement is wrong.
    private static final double kDriveReduction = 6.67 * 9 / 10;
    private static final double driveEncoderDistancePerTurn = kWheelDiameterM * Math.PI / kDriveReduction;
    // andymark ma3 encoder is 1:1
    private static final double turningGearRatio = 1.0;

    public static AMCANSwerveModule100 get(
            String name,
            double currentLimit,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            AnalogTurningEncoder.Drive turningDrive,
            SwerveKinodynamics kinodynamics) {

        VelocityServo<Distance> driveServo = driveServo(
                name,
                currentLimit,
                driveMotorCanId);

        PositionServo<Angle> turningServo = turningServo(
                name,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                turningDrive,
                kinodynamics);

        return new AMCANSwerveModule100(name, driveServo, turningServo);

    }

    private static VelocityServo<Distance> driveServo(
            String name,
            double currentLimit,
            int driveMotorCanId) {
        FalconDriveMotor driveMotor = new FalconDriveMotor(
                name,
                driveMotorCanId,
                currentLimit,
                kDriveReduction,
                kWheelDiameterM);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(
                name,
                driveMotor,
                driveEncoderDistancePerTurn);
        PIDController driveController = new PIDController( //
                0.1, // kP
                0, // kI
                0); // kD
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.0, // kS
                .5); // kV
        return new SelectableVelocityServo<>(
                drive(name),
                driveMotor,
                driveEncoder,
                driveController,
                driveFeedforward);
    }

    private static PositionServo<Angle> turningServo(
            String name,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            AnalogTurningEncoder.Drive turningDrive,
            SwerveKinodynamics kinodynamics) {
        CANTurningMotor turningMotor = new CANTurningMotor(name, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                name,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio, turningDrive);
        PIDController angleVelocityController = new PIDController(
                5, // kP
                0, // kI
                0, // kD
                dt); // dt
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                .25, // kS
                0.015); // kV
        VelocityServo<Angle> turningVelocityServo = new SelectableVelocityServo<>(
                turning(name),
                turningMotor,
                turningEncoder,
                angleVelocityController,
                turningFeedforward);
        PIDController turningPositionController = new PIDController(
                5, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);
        Profile100 profile = kinodynamics.getSteeringProfile();
        PositionServo<Angle> turningServo = new PositionServo<>(
                turning(name),
                turningVelocityServo,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile,
                Angle.instance);
        turningServo.reset();
        return turningServo;
    }

    private AMCANSwerveModule100(
            String name,
            VelocityServo<Distance> driveServo,
            PositionServo<Angle> turningServo) {
        super(name, driveServo, turningServo);
        //
    }
}

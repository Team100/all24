package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.drive.Talon6DriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.motor.turning.CANTurningMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

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

    public static AMCANSwerveModule100 get(
            String name,
            Logger parent,
            double currentLimit,
            double statorLimit,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            EncoderDrive turningDrive,
            SwerveKinodynamics kinodynamics) {
        Logger moduleLogger = parent.child(name);
        PIDConstants drivePidConstants = new PIDConstants(0.05);
        Feedforward100 ff = Feedforward100.makeAMSwerveDriveFalcon6();
        VelocityServo<Distance100> driveServo = driveServo(
                moduleLogger.child("Drive"),
                currentLimit,
                statorLimit,
                driveMotorCanId,
                drivePidConstants,
                ff);

        AngularPositionServo turningServo = turningServo(
                moduleLogger.child("Turning"),
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                turningDrive,
                kinodynamics);

        return new AMCANSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance100> driveServo(
            Logger parent,
            double currentLimit,
            double statorLimit,
            int driveMotorCanId,
            PIDConstants pidConstants,
            Feedforward100 ff) {
        double distancePerTurn = kWheelDiameterM * Math.PI / kDriveReduction;
        Falcon6DriveMotor driveMotor = new Falcon6DriveMotor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                currentLimit,
                statorLimit,
                kDriveReduction,
                kWheelDiameterM,
                pidConstants,
                ff);
        Talon6DriveEncoder driveEncoder = new Talon6DriveEncoder(
                parent,
                driveMotor,
                distancePerTurn);
        return new OutboardVelocityServo<>(
                parent,
                driveMotor,
                driveEncoder);
    }

    private static OnboardAngularPositionServo turningServo(
            Logger parent,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            EncoderDrive turningDrive,
            SwerveKinodynamics kinodynamics) {
        CANTurningMotor turningMotor = new CANTurningMotor(parent, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                parent,
                turningEncoderChannel,
                turningOffset,
                turningDrive);
        PIDController turningPositionController = new PIDController(
                5, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);
        Profile100 profile = kinodynamics.getSteeringProfile();
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
                parent,
                turningMotor,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile);
        turningServo.reset();
        return turningServo;
    }

    private AMCANSwerveModule100(
            String name,
            VelocityServo<Distance100> driveServo,
            AngularPositionServo turningServo) {
        super(name, driveServo, turningServo);
    }
}

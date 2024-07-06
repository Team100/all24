package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.drive.Talon6DriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.motor.turning.TurningMotorController100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * Stock AndyMark module with Falcon drive and PWM 775 steering.
 */
public class AMSwerveModule100 extends SwerveModule100 {
    // AndyMark Swerve & Steer has 4 inch wheel
    private static final double kWheelDiameterM = 0.09628;
    // see andymark.com/products/swerve-and-steer
    private static final double kDriveReduction = 6.67;

    /** @param name like "front left" or whatever */
    public static AMSwerveModule100 get(
            String name,
            Logger parent,
            double currentLimit,
            double statorLimit,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            PIDConstants pidConstants,
            Feedforward100 ff) {
        Logger moduleLogger = parent.child(name);
        LinearVelocityServo driveServo = driveServo(
                moduleLogger.child("Drive"),
                currentLimit,
                statorLimit,
                driveMotorCanId,
                pidConstants,
                ff);

        AngularPositionServo turningServo = turningServo(
                moduleLogger.child("Turning"),
                turningMotorChannel,
                turningEncoderChannel,
                turningOffset,
                kinodynamics);

        return new AMSwerveModule100(name, driveServo, turningServo);
    }

    private static LinearVelocityServo driveServo(
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
        return new OutboardLinearVelocityServo(
                parent,
                driveMotor,
                driveEncoder);
    }

    private static AngularPositionServo turningServo(
            Logger parent,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics) {
        TurningMotorController100 turningMotor = new TurningMotorController100(
                parent,
                new VictorSP(turningMotorChannel),
                turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                parent,
                turningEncoderChannel,
                turningOffset,
                EncoderDrive.DIRECT);

        PIDController turningPositionController = new PIDController(
                0.5, // kP
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

    private AMSwerveModule100(
            String name,
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(name, driveServo, turningServo);
        //
    }

}

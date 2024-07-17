package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.encoder.VelocityBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.LinearMechanismInterface;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.TalonSRXMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.controller.PIDController;

/**
 * For outboard closed-loop control.
 */
public class AMCANSwerveModule100 extends SwerveModule100 {

    /**
     * There is a planetary gearbox between the motor and the steering gear, and the
     * final is 48/40.
     */
    private static final double kSteeringReduction = 71.0 * 40 / 48;
    // AndyMark Swerve & Steer has 4 inch wheel
    private static final double kWheelDiameterM = 0.1016;
    // see andymark.com/products/swerve-and-steer
    // the true value is 6.67 but the measured value is 90% of that,
    // maybe because the wheel measurement is wrong.
    private static final double kDriveReduction = 6.67 * 9 / 10;

    public static AMCANSwerveModule100 get(
            String name,
            SupplierLogger parent,
            double currentLimit,
            double statorLimit,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            EncoderDrive turningDrive,
            SwerveKinodynamics kinodynamics) {
        SupplierLogger moduleLogger = parent.child(name);
        PIDConstants drivePidConstants = new PIDConstants(0.05);
        Feedforward100 ff = Feedforward100.makeAMSwerveDriveFalcon6();
        LinearVelocityServo driveServo = driveServo(
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

    private static LinearVelocityServo driveServo(
            SupplierLogger parent,
            double currentLimit,
            double statorLimit,
            int driveMotorCanId,
            PIDConstants pidConstants,
            Feedforward100 ff) {
        Falcon6Motor driveMotor = new Falcon6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                currentLimit,
                statorLimit,
                pidConstants,
                ff);
        LinearMechanismInterface mech = new LinearMechanism(
                driveMotor, new Talon6Encoder(parent, driveMotor),
                kDriveReduction,
                kWheelDiameterM);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static OnboardAngularPositionServo turningServo(
            SupplierLogger parent,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            EncoderDrive turningDrive,
            SwerveKinodynamics kinodynamics) {
        TalonSRXMotor turningMotor = new TalonSRXMotor(parent, turningMotorCanId);
        // encoder is 1:1 with mech
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
        RotaryMechanism mech = new RotaryMechanism(
                turningMotor,
                new VelocityBareEncoder(parent, turningMotor),
                kSteeringReduction);
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
                parent,
                mech,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile);
        turningServo.reset();
        return turningServo;
    }

    private AMCANSwerveModule100(
            String name,
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(name, driveServo, turningServo);
    }
}

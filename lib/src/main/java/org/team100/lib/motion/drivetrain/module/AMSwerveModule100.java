package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.drive.Drive;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.motor.turning.PWMTurningMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

/**
 * Stock AndyMark module with Falcon drive and PWM 775 steering.
 */
public class AMSwerveModule100 extends SwerveModule100 {
    // AndyMark Swerve & Steer has 4 inch wheel
    private static final double kWheelDiameterM = 0.09628;
    // see andymark.com/products/swerve-and-steer
    private static final double kDriveReduction = 6.67;
    // andymark ma3 encoder is 1:1
    private static final double turningGearRatio = 1.0;

    /** @param name like "front left" or whatever */
    public static AMSwerveModule100 get(
            String name,
            double currentLimit,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            PIDConstants pidConstants,
            FeedforwardConstants feedforwardConstants) {

        VelocityServo<Distance100> driveServo = driveServo(
                name + "/Drive",
                currentLimit,
                driveMotorCanId,
                pidConstants,
                feedforwardConstants);

        PositionServoInterface<Angle100> turningServo = turningServo(
                name + "/Turning",
                turningMotorChannel,
                turningEncoderChannel,
                turningOffset,
                kinodynamics);

        return new AMSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance100> driveServo(String name,
            double currentLimit,
            int driveMotorCanId,
            PIDConstants pidConstants,
            FeedforwardConstants feedforwardConstants) {
        MotorWithEncoder100<Distance100> driveMotor = new Falcon6DriveMotor(
                name,
                driveMotorCanId,
                true,
                currentLimit,
                kDriveReduction,
                kWheelDiameterM,
                pidConstants,
                feedforwardConstants);
        return new OutboardVelocityServo<>(
                name,
                driveMotor,
                driveMotor);
    }

    private static PositionServoInterface<Angle100> turningServo(
            String name,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics) {
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(
                name,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio,
                Drive.DIRECT);

        PIDController turningPositionController = new PIDController(
                0.5, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);
        Profile100 profile = kinodynamics.getSteeringProfile();
        PositionServoInterface<Angle100> turningServo = new PositionServo<>(
                name,
                turningMotor,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile,
                Angle100.instance);
        turningServo.reset();
        return turningServo;
    }

    private AMSwerveModule100(
            String name,
            VelocityServo<Distance100> driveServo,
            PositionServoInterface<Angle100> turningServo) {
        super(name, driveServo, turningServo);
        //
    }

}

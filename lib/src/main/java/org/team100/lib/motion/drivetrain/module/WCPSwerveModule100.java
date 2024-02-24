package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.Drive;
import org.team100.lib.encoder.turning.DutyCycleTurningEncoder;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.motor.turning.Falcon6TurningMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;

public class WCPSwerveModule100 extends SwerveModule100 {
    /**
     * Flipped belt ratios.
     */
    public enum DriveRatio {
        FAST(5.5), MEDIUM(6.55);

        private double m_ratio;

        DriveRatio(double ratio) {
            m_ratio = ratio;
        }
    }

    private static final String m_name = Names.name(WCPSwerveModule100.class);

    // WCP 4 inch wheel
    private static final double kWheelDiameterM = 0.1015;
    // flipped belt ratios
    private static final double kDriveReductionFast = 5.50;
    private static final double kDriveReductionMedium = 6.55;

    /**
     * @param name                  like "front left" or whatever
     * @param curerntLimit          in amps
     * @param driveMotorCanId
     * @param encoderClass          select the type of encoder that exists on the
     *                              robot
     * @param turningMotorCanId
     * @param turningEncoderChannel
     * @param turningOffset
     * @param kinodynamics
     */
    public static WCPSwerveModule100 get(
            String name,
            double currentLimit,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends Encoder100<Angle100>> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            Drive drive,
            MotorPhase motorPhase) {
        name = m_name + "/" + name;
        PIDConstants drivePidConstants = new PIDConstants(.2);
        PIDConstants turningPidConstants = new PIDConstants(0.22); // 5
        FeedforwardConstants turningFeedforwardConstants = FeedforwardConstants.makeWCPSwerveTurningFalcon6();
        FeedforwardConstants driveFeedforwardConstants = FeedforwardConstants.makeWCPSwerveDriveFalcon6();

        VelocityServo<Distance100> driveServo = driveServo(
                name + "/Drive",
                currentLimit,
                driveMotorCanId,
                ratio,
                drivePidConstants,
                driveFeedforwardConstants);

        PositionServoInterface<Angle100> turningServo = turningServo(
                name + "/Turning",
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                10.29,
                kinodynamics,
                drive,
                motorPhase,
                turningPidConstants,
                turningFeedforwardConstants);

        return new WCPSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance100> driveServo(
            String name,
            double currentLimit,
            int driveMotorCanId,
            DriveRatio ratio,
            PIDConstants pidConstants,
            FeedforwardConstants feedforwardConstants) {
                
        MotorWithEncoder100<Distance100> driveMotor = new Falcon6DriveMotor(
                name,
                driveMotorCanId,
                true,
                currentLimit,
                ratio.m_ratio,
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
            Class<? extends Encoder100<Angle100>> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            Drive drive,
            MotorPhase motorPhase,
            PIDConstants lowLevelPID,
            FeedforwardConstants lowLevelFeedforward) {
        final double turningGearRatio = 1.0;
        Motor100<Angle100> turningMotor = new Falcon6TurningMotor(
                name,
                turningMotorCanId,
                motorPhase,
                gearRatio,
                lowLevelPID,
                lowLevelFeedforward);
        Encoder100<Angle100> turningEncoder = turningEncoder(
                encoderClass,
                name,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio,
                drive);
        PIDController turningPositionController = new PIDController(
                1.9, // kP
                0.06, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, -Math.PI);
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

    private static Encoder100<Angle100> turningEncoder(
            Class<?> encoderClass,
            String name,
            int channel,
            double inputOffset,
            double gearRatio,
            Drive drive) {
        if (encoderClass == AnalogTurningEncoder.class) {
            return new AnalogTurningEncoder(name,
                    channel,
                    inputOffset,
                    gearRatio,
                    drive);
        }
        if (encoderClass == DutyCycleTurningEncoder.class) {
            return new DutyCycleTurningEncoder(name,
                    channel,
                    inputOffset,
                    gearRatio,
                    drive);
        }
        throw new IllegalArgumentException("unknown encoder class: " + encoderClass.getName());

    }

    private WCPSwerveModule100(
            String name,
            VelocityServo<Distance100> driveServo,
            PositionServoInterface<Angle100> turningServo) {
        super(name, driveServo, turningServo);
        //
    }
}

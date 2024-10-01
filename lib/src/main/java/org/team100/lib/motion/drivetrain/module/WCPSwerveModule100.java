package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.DutyCycleRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.Profile100;

import edu.wpi.first.math.controller.PIDController;

public class WCPSwerveModule100 extends SwerveModule100 {
    private static final boolean USE_OUTBOARD_STEERING = false;
    private static final double kSteeringSupplyLimit = 10;
    private static final double kSteeringStatorLimit = 20;
    /**
     * WCP calls this "rotation ratio" here, we use the "flipped belt" which is the
     * fastest steering ratio.
     * 12t -> 24t
     * 14t -> 72t
     * = 72 / 7
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    private static final double kSteeringRatio = 10.28571429;

    /**
     * Flipped belt ratios.
     * 
     * See
     * https://docs.wcproducts.com/wcp-swervex/misc/other-configurations/ratio-options
     */
    public enum DriveRatio {
        FAST(5.5),
        MEDIUM(6.55);

        private double m_ratio;

        DriveRatio(double ratio) {
            m_ratio = ratio;
        }
    }

    // WCP 4 inch wheel
    private static final double kWheelDiameterM = 0.0975; // 0.1015

    public static WCPSwerveModule100 getKrakenDrive(
            SupplierLogger2 parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {

        LinearVelocityServo driveServo = driveKrakenServo(
                parent.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio);

        AngularPositionServo turningServo = turningServo(
                parent.child("Turning"),
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                kSteeringRatio,
                kinodynamics,
                drive,
                motorPhase);

        return new WCPSwerveModule100(driveServo, turningServo);
    }

    /**
     * MAKE SURE THAT THE BEVELS ON THE WHEELS FOR ZEROING GO TO THE LEFT
     */
    public static WCPSwerveModule100 getFalconDrive(
            SupplierLogger2 parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {

        LinearVelocityServo driveServo = driveFalconServo(
                parent.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio);

        AngularPositionServo turningServo = turningServo(
                parent.child("Turning"),
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                kSteeringRatio,
                kinodynamics,
                drive,
                motorPhase);

        return new WCPSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo driveKrakenServo(
            SupplierLogger2 parent,
            double supplyLimit,
            double statorLimit,
            int driveMotorCanId,
            DriveRatio ratio) {
        Feedforward100 ff = Feedforward100.makeWCPSwerveDriveFalcon6();
        PIDConstants pid = new PIDConstants(0.2);
        Kraken6Motor driveMotor = new Kraken6Motor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                pid,
                ff);
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new Talon6Encoder(parent, driveMotor),
                ratio.m_ratio,
                kWheelDiameterM);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static LinearVelocityServo driveFalconServo(
        SupplierLogger2 parent,
        double supplyLimit,
        double statorLimit,
        int driveMotorCanId,
        DriveRatio ratio) {
    Feedforward100 ff = Feedforward100.makeWCPSwerveDriveFalcon6();
    PIDConstants pid = new PIDConstants(0.2);
    Falcon6Motor driveMotor = new Falcon6Motor(
            parent,
            driveMotorCanId,
            MotorPhase.FORWARD,
            supplyLimit,
            statorLimit,
            pid,
            ff);
    LinearMechanism mech = new SimpleLinearMechanism(
            driveMotor,
            new Talon6Encoder(parent, driveMotor),
            ratio.m_ratio,
            kWheelDiameterM);
    return new OutboardLinearVelocityServo(
            parent,
            mech);
    }

    private static AngularPositionServo turningServo(
            SupplierLogger2 parent,
            Class<? extends RotaryPositionSensor> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {

        PIDConstants lowLevelPID = null;
        if (USE_OUTBOARD_STEERING) {
            // Talon outboard positional PID
            lowLevelPID = new PIDConstants(10.0, 0.0, 0.0);
        } else {
            // These parameters are handed to Talon outboard velocity PID
            // this seems more likely to oscillate
            // this is tuned in air, not on carpet, so it's probably too soft.
            lowLevelPID = new PIDConstants(0.3, 0.0, 0.0);
        }

        // java uses this to calculate feedforward voltages from target velocities etc
        Feedforward100 ff = Feedforward100.makeWCPSwerveTurningFalcon6();

        Falcon6Motor turningMotor = new Falcon6Motor(
                parent,
                turningMotorCanId,
                motorPhase,
                kSteeringSupplyLimit,
                kSteeringStatorLimit,
                lowLevelPID,
                ff);

        RotaryPositionSensor turningEncoder = turningEncoder(
                encoderClass,
                parent,
                turningEncoderChannel,
                turningOffset,
                drive);

        Profile100 profile = kinodynamics.getSteeringProfile();

        Talon6Encoder builtInEncoder = new Talon6Encoder(
                parent,
                turningMotor);

        RotaryMechanism mech = new RotaryMechanism(
                parent,
                turningMotor,
                builtInEncoder,
                gearRatio);

        if (USE_OUTBOARD_STEERING) {
            AngularPositionServo turningServo = getOutboard(
                    parent,
                    turningEncoder,
                    profile,
                    mech);
            turningServo.reset();
            return turningServo;
        }

        AngularPositionServo turningServo = getOnboard(
                parent,
                kinodynamics,
                turningEncoder,
                profile,
                mech);
        turningServo.reset();
        return turningServo;

    }

    private static AngularPositionServo getOutboard(
            SupplierLogger2 parent,
            RotaryPositionSensor turningEncoder,
            Profile100 profile,
            RotaryMechanism mech) {
        CombinedEncoder combinedEncoder = new CombinedEncoder(
                parent,
                turningEncoder,
                mech);
        AngularPositionServo servo = new OutboardAngularPositionServo(
                parent,
                mech,
                combinedEncoder);
        servo.setProfile(profile);
        return servo;
    }

    private static AngularPositionServo getOnboard(
            SupplierLogger2 parent,
            SwerveKinodynamics kinodynamics,
            RotaryPositionSensor turningEncoder,
            Profile100 profile,
            RotaryMechanism mech) {
        // This is the top-level position controller
        // this is tuned in air, not on carpet, so it's probably too soft.
        PIDController onboardPositionController = new PIDController(
                20, // kP
                0, // kI
                0, // kD
                TimedRobot100.LOOP_PERIOD_S);
        onboardPositionController.enableContinuousInput(-Math.PI, Math.PI);
        onboardPositionController.setTolerance(0.02, 0.02);
        AngularPositionServo servo = new OnboardAngularPositionServo(
                parent,
                mech,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                onboardPositionController);
        servo.setProfile(profile);
        return servo;
    }

    private static RotaryPositionSensor turningEncoder(
            Class<? extends RotaryPositionSensor> encoderClass,
            SupplierLogger2 parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        if (encoderClass == AnalogTurningEncoder.class) {
            return new AnalogTurningEncoder(
                    parent,
                    channel,
                    inputOffset,
                    drive);
        }
        if (encoderClass == DutyCycleRotaryPositionSensor.class) {
            return new AS5048RotaryPositionSensor(
                    parent,
                    channel,
                    inputOffset,
                    drive);
        }
        throw new IllegalArgumentException("unknown encoder class: " + encoderClass.getName());

    }

    private WCPSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(driveServo, turningServo);
        //
    }
}

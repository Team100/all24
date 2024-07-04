package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.drive.Talon6DriveEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.DutyCycleTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.encoder.turning.Talon6TurningEncoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.components.OnboardPositionServo;
import org.team100.lib.motion.components.OutboardPositionServo;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.SelectablePositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.drive.Kraken6DriveMotor;
import org.team100.lib.motor.turning.Falcon6TurningMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

public class WCPSwerveModule100 extends SwerveModule100 {
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

    public static WCPSwerveModule100 get(
            String name,
            Logger parent,
            double supplyLimitAmps,
            double statorLimitAmps,
            int driveMotorCanId,
            DriveRatio ratio,
            Class<? extends Encoder100<Angle100>> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase) {
        PIDConstants drivePidConstants = new PIDConstants(.2); // .2
        PIDConstants turningPidConstants = new PIDConstants(.32); // 5
        Feedforward100 turningFF = Feedforward100.makeWCPSwerveTurningFalcon6();
        Feedforward100 driveFF = Feedforward100.makeWCPSwerveDriveFalcon6();

        VelocityServo<Distance100> driveServo = driveServo(
                parent.child("Drive"),
                supplyLimitAmps,
                statorLimitAmps,
                driveMotorCanId,
                ratio,
                drivePidConstants,
                driveFF);

        PositionServo<Angle100> turningServo = turningServo(
                parent.child("Turning"),
                encoderClass,
                turningMotorCanId,
                turningEncoderChannel,
                turningOffset,
                10.29,
                kinodynamics,
                drive,
                motorPhase,
                turningPidConstants,
                turningFF);

        return new WCPSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance100> driveServo(
            Logger parent,
            double supplyLimit,
            double statorLimit,
            int driveMotorCanId,
            DriveRatio ratio,
            PIDConstants pidConstants,
            Feedforward100 ff) {
        double distancePerTurn = kWheelDiameterM * Math.PI / ratio.m_ratio;

        Kraken6DriveMotor driveMotor = new Kraken6DriveMotor(
                parent,
                driveMotorCanId,
                MotorPhase.FORWARD,
                supplyLimit,
                statorLimit,
                ratio.m_ratio,
                kWheelDiameterM,
                pidConstants,
                ff);
        Talon6DriveEncoder encoder = new Talon6DriveEncoder(
                parent,
                driveMotor,
                distancePerTurn);
        return new OutboardVelocityServo<>(
                parent,
                driveMotor,
                encoder);
    }


    private static PositionServo<Angle100> turningServo(
            Logger parent,
            Class<? extends Encoder100<Angle100>> encoderClass,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double gearRatio,
            SwerveKinodynamics kinodynamics,
            EncoderDrive drive,
            MotorPhase motorPhase,
            PIDConstants lowLevelPID,
            Feedforward100 ff) {
        final double turningGearRatio = 1.0;
        Falcon6TurningMotor turningMotor = new Falcon6TurningMotor(
                parent,
                turningMotorCanId,
                motorPhase,
                gearRatio,
                lowLevelPID,
                ff);
        Encoder100<Angle100> turningEncoder = turningEncoder(
                encoderClass,
                parent,
                turningEncoderChannel,
                turningOffset,
                turningGearRatio,
                drive);
        PIDController turningPositionController = new PIDController(
                10, // kP
                0.06, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, -Math.PI);
        turningPositionController.setTolerance(0.1, 0.1);

            Profile100 profile = kinodynamics.getSteeringProfile();
            
        PositionServo<Angle100> turningServo = getTurningServo(
                parent,
                kinodynamics,
                turningMotor,
                turningEncoder,
                turningGearRatio,
                turningPositionController,
                profile);
        turningServo.reset();
        return turningServo;
    }

    private static PositionServo<Angle100> getTurningServo(
            Logger parent,
            SwerveKinodynamics kinodynamics,
            Falcon6TurningMotor turningMotor,
            Encoder100<Angle100> turningEncoder,
            double turningGearRatio,
            PIDController turningPositionController,
            Profile100 profile) {
        Talon6TurningEncoder builtInEncoder = new Talon6TurningEncoder(
                parent,
                turningMotor,
                turningGearRatio);
        // if we correct to exactly the primary reading, we effectively inject noise
        // into the secondary, so soften the response.
        final double primaryAuthority = 0.1;
        CombinedEncoder<Angle100> combinedEncoder = new CombinedEncoder<>(
                turningEncoder,
                primaryAuthority,
                builtInEncoder);
        PositionServo<Angle100> outboard = new OutboardPositionServo<>(
                parent,
                turningMotor,
                combinedEncoder,
                profile,
                Angle100.instance);
        PositionServo<Angle100> onboard = new OnboardPositionServo<>(
                parent,
                turningMotor,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile,
                Angle100.instance);
        return new SelectablePositionServo<>(
                outboard,
                onboard,
                () -> Experiments.instance.enabled(Experiment.OutboardSteering));
    }

    private static Encoder100<Angle100> turningEncoder(
            Class<?> encoderClass,
            Logger parent,
            int channel,
            double inputOffset,
            double gearRatio,
            EncoderDrive drive) {
        if (encoderClass == AnalogTurningEncoder.class) {
            return new AnalogTurningEncoder(
                    parent,
                    channel,
                    inputOffset,
                    gearRatio,
                    drive);
        }
        if (encoderClass == DutyCycleTurningEncoder.class) {
            return new DutyCycleTurningEncoder(
                    parent,
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
            PositionServo<Angle100> turningServo) {
        super(name, driveServo, turningServo);
        //
    }
}

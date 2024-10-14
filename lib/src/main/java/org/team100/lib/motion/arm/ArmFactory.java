package org.team100.lib.motion.arm;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AnalogTurningEncoder;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;

/**
 * Produces real or simulated arm subsystems depending on identity.
 */
public class ArmFactory {
    private static final String kArm = "arm";
    private static final String kLower = "arm/lower";
    private static final String kUpper = "arm/upper";

    public static ArmSubsystem get(LoggerFactory parent) {
        switch (Identity.instance) {
            case TEST_BOARD_6B:
                return real(parent);
            case BLANK:
            default:
                return simulated(parent);
        }
    }

    private static ArmSubsystem real(LoggerFactory parent) {
        final double kLowerEncoderOffset = 0.861614;
        final double kUpperEncoderOffset = 0.266396;
        final double kReduction = 600;

        LoggerFactory lowerLogger = parent.child(kLower);
        CANSparkMotor lowerMotor = new NeoCANSparkMotor(
                lowerLogger,
                4,
                MotorPhase.FORWARD,
                8,
                Feedforward100.makeNeo(),
                new PIDConstants(0, 0, 0));
        RotaryMechanism lowerMech = new SimpleRotaryMechanism(
                lowerLogger,
                lowerMotor,
                new CANSparkEncoder(lowerLogger, lowerMotor),
                kReduction);

        // NOTE: the encoder inversion used to be in the subsystem,
        // but now it is here.
        RotaryPositionSensor lowerEncoder = new AnalogTurningEncoder(
                lowerLogger,
                1, // analog input 1
                kLowerEncoderOffset,
                EncoderDrive.INVERSE);

        LoggerFactory upperLogger = parent.child(kUpper);
        CANSparkMotor upperMotor = new NeoCANSparkMotor(
                upperLogger,
                30,
                MotorPhase.FORWARD,
                1,
                Feedforward100.makeNeo(),
                new PIDConstants(0, 0, 0));
        RotaryMechanism upperMech = new SimpleRotaryMechanism(
                upperLogger,
                upperMotor,
                new CANSparkEncoder(upperLogger, upperMotor),
                kReduction);
        RotaryPositionSensor upperEncoder = new AnalogTurningEncoder(
                upperLogger,
                0, // analog input 0
                kUpperEncoderOffset,
                EncoderDrive.DIRECT);

        return new ArmSubsystem(
                parent.child(kArm),
                lowerMech,
                lowerEncoder,
                upperMech,
                upperEncoder);
    }

    private static ArmSubsystem simulated(LoggerFactory parent) {
        // for testing
        // note very high reduction ratio
        final double kFreeSpeedRad_S = 200;
        final double kReduction = 600;
        // motor speed is rad/s

        LoggerFactory lowerLogger = parent.child(kLower);
        SimulatedBareMotor lowerMotor = new SimulatedBareMotor(lowerLogger, kFreeSpeedRad_S);
        RotaryMechanism lowerMech = new SimpleRotaryMechanism(
                lowerLogger,
                lowerMotor,
                new SimulatedBareEncoder(lowerLogger, lowerMotor),
                kReduction);
        // limits used to be -1, 1, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor lowerEncoder = new SimulatedRotaryPositionSensor(
                lowerLogger, lowerMech);

        LoggerFactory upperLogger = parent.child(kUpper);
        SimulatedBareMotor upperMotor = new SimulatedBareMotor(upperLogger, kFreeSpeedRad_S);
        RotaryMechanism upperMech = new SimpleRotaryMechanism(
                upperLogger,
                upperMotor,
                new SimulatedBareEncoder(upperLogger, upperMotor),
                kReduction);
        // limits used to be 0.1, 2.5, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor upperEncoder = new SimulatedRotaryPositionSensor(
                upperLogger, upperMech);
        return new ArmSubsystem(
                parent.child(kArm),
                lowerMech,
                lowerEncoder,
                upperMech,
                upperEncoder);
    }

    private ArmFactory() {
        //
    }

}

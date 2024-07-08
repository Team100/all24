package org.team100.lib.motion.arm;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.telemetry.Logger;

/**
 * Produces real or simulated arm subsystems depending on identity.
 */
public class ArmFactory {
    private static final String kArm = "arm";
    private static final String kLower = "arm/lower";
    private static final String kUpper = "arm/upper";

    public static ArmSubsystem get(Logger parent) {
        switch (Identity.instance) {
            case TEST_BOARD_6B:
                return real(parent);
            case BLANK:
            default:
                return simulated(parent);
        }
    }

    private static ArmSubsystem real(Logger parent) {
        final double kLowerEncoderOffset = 0.861614;
        final double kUpperEncoderOffset = 0.266396;
        final double kReduction = 600;

        BareMotor lowerMotor = new NeoCANSparkMotor(
                parent.child(kLower),
                4,
                MotorPhase.FORWARD,
                8,
                Feedforward100.makeNeo(),
                new PIDConstants(0, 0, 0));
        RotaryMechanism lowerMech = new RotaryMechanism(lowerMotor, kReduction);

        // NOTE: the encoder inversion used to be in the subsystem,
        // but now it is here.
        RotaryPositionSensor lowerEncoder = new AnalogTurningEncoder(
                parent.child(kLower),
                1, // analog input 1
                kLowerEncoderOffset,
                EncoderDrive.INVERSE);

        BareMotor upperMotor = new NeoCANSparkMotor(
                parent.child(kUpper),
                30,
                MotorPhase.FORWARD,
                1,
                Feedforward100.makeNeo(),
                new PIDConstants(0, 0, 0));
        RotaryMechanism upperMech = new RotaryMechanism(upperMotor, kReduction);
        RotaryPositionSensor upperEncoder = new AnalogTurningEncoder(
                parent.child(kUpper),
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

    private static ArmSubsystem simulated(Logger parent) {
        // for testing
        // note very high reduction ratio
        final double kFreeSpeedRad_S = 200;
        final double kReduction = 600;
        // motor speed is rad/s

        SimulatedBareMotor lowerMotor = new SimulatedBareMotor(parent.child(kLower), kFreeSpeedRad_S);
        RotaryMechanism lowerMech = new RotaryMechanism(lowerMotor, kReduction);
        // limits used to be -1, 1, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor lowerEncoder = new SimulatedRotaryPositionSensor(
                parent.child(kLower), lowerMech);

        SimulatedBareMotor upperMotor = new SimulatedBareMotor(parent.child(kUpper), kFreeSpeedRad_S);
        RotaryMechanism upperMech = new RotaryMechanism(upperMotor, kReduction);
        // limits used to be 0.1, 2.5, when we used winding encoders.
        // i don't think we ever actually *use* the limits for anything, though.
        SimulatedRotaryPositionSensor upperEncoder = new SimulatedRotaryPositionSensor(
                parent.child(kUpper), upperMech);
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

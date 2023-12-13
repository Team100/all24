package org.team100.lib.motion.arm;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.turning.AnalogTurningEncoder;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.arm.JointMotor;
import org.team100.lib.units.Angle;

public class ArmFactory {
    private static final String kArm = "/arm";
    private static final String kLower = "/arm/lower";
    private static final String kUpper = "/arm/upper";

    public static ArmSubsystem get() {
        Identity identity = Identity.get();
        switch (identity) {
            case TEST_BOARD_6B:
                // TODO: use the correct identity.
                return real();
            case BLANK:
                // for testing
                return simulated();
            default:
                return real();
        }
    }

    private static ArmSubsystem real() {
        Motor100<Angle> lowerMotor = new JointMotor(kLower, 4, 8);
        Encoder100<Angle> lowerEncoder = new AnalogTurningEncoder(
                kLower, 1, 0, 1, AnalogTurningEncoder.Drive.DIRECT);
        Motor100<Angle> upperMotor = new JointMotor(kUpper, 30, 1);
        Encoder100<Angle> upperEncoder = new AnalogTurningEncoder(
                kUpper, 0, 0, 1, AnalogTurningEncoder.Drive.DIRECT);
        return new ArmSubsystem(kArm, lowerMotor, lowerEncoder, upperMotor, upperEncoder);
    }

    private static ArmSubsystem simulated() {
        // for testing
        // note very high reduction ratio
        SimulatedMotor<Angle> lowerMotor = new SimulatedMotor<>(kLower);
        SimulatedEncoder<Angle> lowerEncoder = new SimulatedEncoder<>(
                Angle.instance, kLower, lowerMotor, 50);
        SimulatedMotor<Angle> upperMotor = new SimulatedMotor<>(kUpper);
        SimulatedEncoder<Angle> upperEncoder = new SimulatedEncoder<>(
                Angle.instance, kUpper, upperMotor, 50);
        return new ArmSubsystem(kArm, lowerMotor, lowerEncoder, upperMotor, upperEncoder);
    }

    private ArmFactory() {
        //
    }

}

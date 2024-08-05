package org.team100.lib.util;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.SimpleLinearMechanism;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.telemetry.SupplierLogger;

public class CollectionUtil {

    public static LinearMechanism getNEO550LinearMechanism(
            String name,
            SupplierLogger parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        SupplierLogger moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
            moduleLogger, 
            canID,
            motorPhase,
            currentLimit,
            Feedforward100.makeNeo550(),
            new PIDConstants(1));
        return getMechanismNoEncoder(
                parent,
                motor,
                gearRatio,
                wheelDiameterM);
    }

    public static RotaryMechanism getNEO550RotaryMechanism(
            String name,
            SupplierLogger parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        SupplierLogger moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
            moduleLogger, 
            canID,
            motorPhase,
            currentLimit,
            Feedforward100.makeNeo550(),
            new PIDConstants(1));
            return new RotaryMechanism(
                parent,
                motor,
                new CANSparkEncoder(parent, motor),
                gearRatio);
    }

    public static LinearVelocityServo getNEO550VelocityServo(
        String name,
        SupplierLogger parent,
        int currentLimit,
        int canID,
        double gearRatio,
        MotorPhase motorPhase,
        double wheelDiameterM
    ) {
        return new OutboardLinearVelocityServo(parent, getNEO550LinearMechanism( name,
        parent,
         currentLimit,
         canID,
         gearRatio,
         motorPhase,
         wheelDiameterM));
    }

    public static LinearVelocityServo simulatedDriveServo(SupplierLogger parent) {
        return new OutboardLinearVelocityServo(
                parent,
                simulatedLinearMechanism(parent));
    }

    public static RotaryMechanism simulatedRotaryMechanism(SupplierLogger parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        return new RotaryMechanism(parent, driveMotor, new SimulatedBareEncoder(parent, driveMotor),1);
    }


    public static LinearMechanism simulatedLinearMechanism(SupplierLogger parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        return new SimpleLinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
    }

    private static LinearMechanism getMechanismNoEncoder(SupplierLogger parent, Neo550CANSparkMotor motor, double gearRatio, double wheelDiameterM) {
            return new SimpleLinearMechanism(
                motor,
                new CANSparkEncoder(parent, motor),
                gearRatio,
                wheelDiameterM);
    }
}

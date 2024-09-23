package org.team100.lib.util;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.logging.SupplierLogger2;

public class Neo550Factory {

    public static LinearMechanism getNEO550LinearMechanism(
            String name,
            SupplierLogger2 parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        SupplierLogger2 moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canID,
                motorPhase,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants(.00000001));
        return new SimpleLinearMechanism(
                motor,
                new CANSparkEncoder(moduleLogger, motor),
                gearRatio,
                wheelDiameterM);
    }

    public static RotaryMechanism getNEO550RotaryMechanism(
            String name,
            SupplierLogger2 parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        SupplierLogger2 moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canID,
                motorPhase,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants(1));
        return new RotaryMechanism(
                moduleLogger,
                motor,
                new CANSparkEncoder(moduleLogger, motor),
                gearRatio);
    }

    public static OutboardGravityServo getNEO550GravityServo(
            String name,
            SupplierLogger2 parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        SupplierLogger2 moduleLogger = parent.child(name);
        Neo550CANSparkMotor driveMotor = new Neo550CANSparkMotor(moduleLogger, canID, motorPhase, currentLimit,
                Feedforward100.makeNeo550(), new PIDConstants(0.0001));
        RotaryMechanism rotaryMechanism = new RotaryMechanism(moduleLogger, driveMotor,
                new CANSparkEncoder(moduleLogger, driveMotor), 1);
        return new OutboardGravityServo(
                new OutboardAngularPositionServo(
                        moduleLogger,
                        rotaryMechanism,
                        new CombinedEncoder(
                                moduleLogger,
                                new SimulatedRotaryPositionSensor(moduleLogger, rotaryMechanism),
                                rotaryMechanism)),
                1,
                1);
    }

    public static LinearVelocityServo getNEO550VelocityServo(
            String name,
            SupplierLogger2 parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        SupplierLogger2 moduleLogger = parent.child(name);
        return new OutboardLinearVelocityServo(moduleLogger, getNEO550LinearMechanism(name,
                moduleLogger,
                currentLimit,
                canID,
                gearRatio,
                motorPhase,
                wheelDiameterM));
    }

    public static OutboardLinearVelocityServo simulatedDriveServo(SupplierLogger2 parent) {
        return new OutboardLinearVelocityServo(
                parent,
                simulatedLinearMechanism(parent));
    }

    public static OutboardGravityServo simulatedGravityServo(SupplierLogger2 parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        RotaryMechanism rotaryMechanism = new RotaryMechanism(parent, driveMotor,
                new SimulatedBareEncoder(parent, driveMotor), 1);
        return new OutboardGravityServo(
                new OutboardAngularPositionServo(
                        parent,
                        rotaryMechanism,
                        new CombinedEncoder(
                                parent,
                                new SimulatedRotaryPositionSensor(parent, rotaryMechanism),
                                rotaryMechanism)),
                1,
                1);
    }

    public static RotaryMechanism simulatedRotaryMechanism(SupplierLogger2 parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        return new RotaryMechanism(parent, driveMotor, new SimulatedBareEncoder(parent, driveMotor), 1);
    }

    public static LinearMechanism simulatedLinearMechanism(SupplierLogger2 parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        return new SimpleLinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
    }
}

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
import org.team100.lib.logging.LoggerFactory;

public class Neo550Factory {

    public static LinearMechanism getNEO550LinearMechanism(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        LoggerFactory moduleLogger = parent.child(name);
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
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        LoggerFactory moduleLogger = parent.child(name);
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
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double p,
            double gravityNm,
            double offsetRad) {
        LoggerFactory moduleLogger = parent.child(name);
        Neo550CANSparkMotor driveMotor = new Neo550CANSparkMotor(moduleLogger, canID, motorPhase, currentLimit,
                Feedforward100.makeNeo550(), new PIDConstants(p));
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
                gravityNm,
                offsetRad);
    }

    public static LinearVelocityServo getNEO550VelocityServo(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        LoggerFactory moduleLogger = parent.child(name);
        return new OutboardLinearVelocityServo(moduleLogger, getNEO550LinearMechanism(name,
                moduleLogger,
                currentLimit,
                canID,
                gearRatio,
                motorPhase,
                wheelDiameterM));
    }

    public static OutboardLinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        return new OutboardLinearVelocityServo(
                parent,
                simulatedLinearMechanism(parent));
    }

    public static OutboardGravityServo simulatedGravityServo(LoggerFactory parent) {
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

    public static RotaryMechanism simulatedRotaryMechanism(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        return new RotaryMechanism(parent, driveMotor, new SimulatedBareEncoder(parent, driveMotor), 1);
    }

    public static LinearMechanism simulatedLinearMechanism(LoggerFactory parent) {
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

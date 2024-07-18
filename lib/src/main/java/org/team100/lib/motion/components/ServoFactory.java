package org.team100.lib.motion.components;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.SimpleLinearMechanism;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.controller.PIDController;

public class ServoFactory {

    public static LimitedLinearVelocityServo limitedNeoVelocityServo(
            SupplierLogger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameter,
            double maxVelocity,
            double maxAccel,
            double maxDecel,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoCANSparkMotor motor = new NeoCANSparkMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                ff,
                lowLevelVelocityConstants);
        CANSparkEncoder encoder = new CANSparkEncoder(
                parent,
                motor);
        LinearMechanism mech = new SimpleLinearMechanism(
                motor,
                encoder,
                gearRatio,
                wheelDiameter);
        LinearVelocityServo v = new OutboardLinearVelocityServo(
                parent,
                mech);
        return new LimitedLinearVelocityServo(v,
                maxVelocity,
                maxAccel,
                maxDecel);
    }

    public static LimitedLinearVelocityServo limitedSimulatedVelocityServo(
            SupplierLogger parent,
            double gearRatio,
            double wheelDiameterM,
            double maxVelocity,
            double maxAccel,
            double maxDecel) {
        // motor speed is rad/s
        BareMotor motor = new SimulatedBareMotor(parent, 600);
        LinearMechanism mech = new SimpleLinearMechanism(
                motor,
                new SimulatedBareEncoder(parent, motor),
                gearRatio,
                wheelDiameterM);
        LinearVelocityServo v = new OutboardLinearVelocityServo(
                parent,
                mech);
        return new LimitedLinearVelocityServo(v,
                maxVelocity,
                maxAccel,
                maxDecel);
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static OnboardAngularPositionServo neoVortexAngleServo(
            SupplierLogger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double maxVelocity,
            double maxAccel,
            PIDController controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        CANSparkMotor motor = new NeoVortexCANSparkMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                ff,
                lowLevelVelocityConstants);
        RotaryMechanism mech = new RotaryMechanism(
                motor,
                new CANSparkEncoder(parent, motor),
                gearRatio);
        RotaryPositionSensor sensor = new ProxyRotaryPositionSensor(mech);
        return new OnboardAngularPositionServo(
                parent,
                mech,
                sensor,
                maxVelocity,
                controller,
                new TrapezoidProfile100(maxVelocity, maxAccel, 0.05));
    }

    public static OnboardAngularPositionServo simulatedAngleServo(
            SupplierLogger parent,
            double maxVelocity,
            double maxAccel,
            PIDController controller) {
        // motor speed is rad/s
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 600);
        RotaryMechanism mech = new RotaryMechanism(
                motor,
                new SimulatedBareEncoder(parent, motor),
                1);
        RotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                parent,
                mech);
        // the new sim doesn't have hard stops; should it?
        // 0, // minimum hard stop
        // 2); // maximum hard stop
        return new OnboardAngularPositionServo(
                parent,
                mech,
                sensor,
                maxVelocity,
                controller,
                new TrapezoidProfile100(maxVelocity, maxAccel, 0.05));
    }

    private ServoFactory() {
        //
    }
}

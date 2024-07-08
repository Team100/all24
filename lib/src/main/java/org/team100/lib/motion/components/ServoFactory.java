package org.team100.lib.motion.components;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.drive.NeoDriveEncoder;
import org.team100.lib.encoder.turning.NeoVortexTurningEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Logger;

import edu.wpi.first.math.controller.PIDController;

public class ServoFactory {

    public static LimitedLinearVelocityServo limitedNeoVelocityServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoCANSparkMotor motor = new NeoCANSparkMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                ff,
                lowLevelVelocityConstants);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                parent,
                motor);
        LinearMechanism mech = new LinearMechanism(
                motor,
                encoder,
                param.gearRatio(),
                param.wheelDiameter());
        LinearVelocityServo v = new OutboardLinearVelocityServo(
                parent,
                mech);
        return new LimitedLinearVelocityServo(v,
                param.maxVelM_S(),
                param.maxAccelM_S2(),
                param.maxDecelM_S2());
    }

    public static LimitedLinearVelocityServo limitedSimulatedVelocityServo(
            Logger parent,
            SysParam param,
            double gearRatio,
            double wheelDiameterM) {
        // motor speed is rad/s
        BareMotor motor = new SimulatedBareMotor(parent, 600);
        LinearMechanism mech = new LinearMechanism(
                motor,
                new SimulatedBareEncoder(parent, motor),
                gearRatio,
                wheelDiameterM);
        LinearVelocityServo v = new OutboardLinearVelocityServo(
                parent,
                mech);
        return new LimitedLinearVelocityServo(v,
                param.maxVelM_S(),
                param.maxAccelM_S2(),
                param.maxDecelM_S2());
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static OnboardAngularPositionServo neoVortexAngleServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoVortexCANSparkMotor motor = new NeoVortexCANSparkMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                ff,
                lowLevelVelocityConstants);
        NeoVortexTurningEncoder encoder = new NeoVortexTurningEncoder(
                parent,
                motor,
                param.gearRatio());
        RotaryMechanism mech = new RotaryMechanism(motor, param.gearRatio());
        RotaryPositionSensor sensor = new ProxyRotaryPositionSensor(encoder);
        return new OnboardAngularPositionServo(
                parent,
                mech,
                sensor,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05));
    }

    public static OnboardAngularPositionServo2 simulatedAngleServo(
            Logger parent,
            SysParam param,
            PIDController controller) {
        // motor speed is rad/s
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 600);
        RotaryMechanism mech = new RotaryMechanism(motor, 1);
        RotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                parent,
                mech);
        // the new sim doesn't have hard stops; should it?
        // 0, // minimum hard stop
        // 2); // maximum hard stop
        return new OnboardAngularPositionServo2(
                parent,
                mech,
                sensor,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05));
    }

    private ServoFactory() {
        //
    }
}

package org.team100.lib.motion.components;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.drive.NeoDriveEncoder;
import org.team100.lib.encoder.drive.NeoVortexDriveEncoder;
import org.team100.lib.encoder.turning.NeoTurningEncoder;
import org.team100.lib.encoder.turning.NeoVortexTurningEncoder;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.motor.drive.NeoVortexDriveMotor;
import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.motor.turning.NeoVortexTurningMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

public class ServoFactory {

    /**
     * 
     * @param name
     * @param canId
     * @param motorPhase
     * @param gearRatio
     * @param wheelDiameter
     * @param maxVel
     * @param maxAccel
     * @param maxDecel      maximum decleration: usually mechanisms can slow down
     *                      faster than they can speed up.
     * @return
     */
    public static LimitedVelocityServo<Distance100> limitedNeoVelocityServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoDriveMotor motor = new NeoDriveMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                param.wheelDiameter(),
                ff,
                lowLevelVelocityConstants);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                parent,
                motor,
                param.wheelDiameter() * Math.PI / param.gearRatio());
        VelocityServo<Distance100> v = new OutboardVelocityServo<>(
                parent,
                motor,
                encoder);
        return new LimitedVelocityServo<>(v,
                param.maxVelM_S(),
                param.maxAccelM_S2(),
                param.maxDecelM_S2());
    }

    public static LimitedVelocityServo<Distance100> limitedSimulatedVelocityServo(
            Logger parent,
            SysParam param) {
        // motor speed is rad/s
        SimulatedMotor<Distance100> motor = new SimulatedMotor<>(parent, 600);
        SimulatedEncoder<Distance100> encoder = new SimulatedEncoder<>(parent, motor, 1, -1, 1);
        VelocityServo<Distance100> v = new OutboardVelocityServo<>(
                parent,
                motor,
                encoder);
        return new LimitedVelocityServo<>(v,
                param.maxVelM_S(),
                param.maxAccelM_S2(),
                param.maxDecelM_S2());
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static PositionServo<Angle100> neoAngleServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDConstants controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoTurningMotor motor = new NeoTurningMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                ff,
                lowLevelVelocityConstants);
        NeoTurningEncoder encoder = new NeoTurningEncoder(
                parent,
                motor,
                param.gearRatio());
        return new OnboardPositionServo<>(
                parent,
                motor,
                encoder,
                param.maxVelM_S(),
                new PIDController(controller.getP(), controller.getI(), controller.getD()),
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Angle100.instance);
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static PositionServo<Angle100> neoVortexAngleServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoVortexTurningMotor motor = new NeoVortexTurningMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                ff,
                lowLevelVelocityConstants);
        NeoVortexTurningEncoder encoder = new NeoVortexTurningEncoder(
                parent,
                motor,
                param.gearRatio());
        return new OnboardPositionServo<>(
                parent,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Angle100.instance);
    }

    public static PositionServo<Angle100> simulatedAngleServo(
            Logger parent,
            SysParam param,
            PIDController controller) {
        // motor speed is rad/s
        SimulatedMotor<Angle100> motor = new SimulatedMotor<>(parent, 600);
        SimulatedEncoder<Angle100> encoder = new SimulatedEncoder<>(

                parent,
                motor,
                1,
                0, // minimum hard stop
                2); // maximum hard stop
        return new OnboardPositionServo<>(
                parent,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Angle100.instance);
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static PositionServo<Distance100> neoDistanceServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoDriveMotor motor = new NeoDriveMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                param.wheelDiameter(),
                ff,
                lowLevelVelocityConstants);
        Encoder100<Distance100> encoder = new NeoDriveEncoder(
                parent,
                motor,
                param.wheelDiameter() * Math.PI / param.gearRatio());
        return new OnboardPositionServo<>(
                parent,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Distance100.instance);
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     * 
     * @param ff in VOLTS VOLTS VOLTS
     */
    public static OnboardPositionServo<Distance100> neoVortexDistanceServo(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            Feedforward100 ff,
            PIDConstants lowLevelVelocityConstants) {
        NeoVortexDriveMotor motor = new NeoVortexDriveMotor(
                parent,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                param.wheelDiameter(),
                ff,
                lowLevelVelocityConstants);
        Encoder100<Distance100> encoder = new NeoVortexDriveEncoder(
                parent,
                motor,
                param.wheelDiameter() * Math.PI / param.gearRatio());
        return new OnboardPositionServo<>(
                parent,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Distance100.instance);
    }

    public static PositionServo<Distance100> simulatedDistanceServo(
            Logger parent,
            SysParam param,
            PIDController controller) {
        // motor speed is rad/s
        SimulatedMotor<Distance100> motor = new SimulatedMotor<>(parent, 600);
        Encoder100<Distance100> encoder = new SimulatedEncoder<>(parent, motor, 1, -1, 1);
        return new OnboardPositionServo<>(
                parent,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Distance100.instance);
    }

    private ServoFactory() {
        //
    }
}

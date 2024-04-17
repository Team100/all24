package org.team100.lib.motion.components;

import org.team100.lib.config.FeedforwardConstants;
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
            String name,
            int canId,
            boolean motorPhase,
            int currentLimit,
            SysParam param,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                param.wheelDiameter(),
                lowLevelFeedforwardConstants,
                lowLevelVelocityConstants);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                name,
                motor,
                param.wheelDiameter() * Math.PI / param.gearRatio());
        VelocityServo<Distance100> v = new OutboardVelocityServo<>(
                name,
                motor,
                encoder);
        return new LimitedVelocityServo<>(v,
                param.maxVelM_S(),
                param.maxAccelM_S2(),
                param.maxDecelM_S2());
    }

    public static LimitedVelocityServo<Distance100> limitedSimulatedVelocityServo(
            String name,
            SysParam param) {
        // motor speed is rad/s
        SimulatedMotor<Distance100> motor = new SimulatedMotor<>(name, 600);
        SimulatedEncoder<Distance100> encoder = new SimulatedEncoder<>(name, motor, 1, -1, 1);
        VelocityServo<Distance100> v = new OutboardVelocityServo<>(
                name,
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
    public static PositionServoInterface<Angle100> neoAngleServo(
            String name,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDConstants controller,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        NeoTurningMotor motor = new NeoTurningMotor(
                name,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                lowLevelFeedforwardConstants,
                lowLevelVelocityConstants);
        NeoTurningEncoder encoder = new NeoTurningEncoder(
                name,
                motor,
                param.gearRatio());

        return new PositionServo<>(
                name,
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
    public static PositionServoInterface<Angle100> neoVortexAngleServo(
            String name,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        NeoVortexTurningMotor motor = new NeoVortexTurningMotor(
                name,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                lowLevelFeedforwardConstants,
                lowLevelVelocityConstants);
        NeoVortexTurningEncoder encoder = new NeoVortexTurningEncoder(
                name,
                motor,
                param.gearRatio());
        return new PositionServo<>(
                name,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Angle100.instance);
    }

    public static PositionServoInterface<Angle100> simulatedAngleServo(
            String name,
            SysParam param,
            PIDController controller) {
        // motor speed is rad/s
        SimulatedMotor<Angle100> motor = new SimulatedMotor<>(name, 600);
        SimulatedEncoder<Angle100> encoder = new SimulatedEncoder<>(
                name,
                motor,
                1,
                0, // minimum hard stop
                2); // maximum hard stop
        return new PositionServo<>(
                name,
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
    public static PositionServoInterface<Distance100> neoDistanceServo(
            String name,
            int canId,
            boolean motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                param.wheelDiameter(),
                lowLevelFeedforwardConstants,
                lowLevelVelocityConstants);
        Encoder100<Distance100> encoder = new NeoDriveEncoder(
                name,
                motor,
                param.wheelDiameter() * Math.PI / param.gearRatio());
        return new PositionServo<>(
                name,
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
     * @param lowLevelFeedforwardConstants in VOLTS VOLTS VOLTS
     */
    public static PositionServo<Distance100> neoVortexDistanceServo(
            String name,
            int canId,
            boolean motorPhase,
            int currentLimit,
            SysParam param,
            PIDController controller,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        NeoVortexDriveMotor motor = new NeoVortexDriveMotor(
                name,
                canId,
                motorPhase,
                currentLimit,
                param.gearRatio(),
                param.wheelDiameter(),
                lowLevelFeedforwardConstants,
                lowLevelVelocityConstants);
        Encoder100<Distance100> encoder = new NeoVortexDriveEncoder(
                name,
                motor,
                param.wheelDiameter() * Math.PI / param.gearRatio());
        return new PositionServo<>(
                name,
                motor,
                encoder,
                param.maxVelM_S(),
                controller,
                new TrapezoidProfile100(param.maxVelM_S(), param.maxAccelM_S2(), 0.05),
                Distance100.instance);
    }

    public static PositionServoInterface<Distance100> simulatedDistanceServo(
            String name,
            SysParam param,
            PIDController controller) {
        // motor speed is rad/s
        SimulatedMotor<Distance100> motor = new SimulatedMotor<>(name, 600);
        Encoder100<Distance100> encoder = new SimulatedEncoder<>(name, motor, 1, -1, 1);
        return new PositionServo<>(
                name,
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

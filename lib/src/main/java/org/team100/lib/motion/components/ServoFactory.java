package org.team100.lib.motion.components;

import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.drive.NeoDriveEncoder;
import org.team100.lib.encoder.turning.NeoTurningEncoder;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.motor.turning.NeoTurningMotor;
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
     * @param maxDecel maximum decleration: usually mechanisms can slow down faster than they can speed up.
     * @return
     */
    public static LimitedVelocityServo<Distance100> limitedNeoVelocityServo(
            String name,
            int canId,
            boolean motorPhase,
            SysParam param) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                param.kGearRatio(),
                param.kWheelDiameter());
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                name,
                motor,
                param.kWheelDiameter() * Math.PI);
        VelocityServo<Distance100> v = new OutboardVelocityServo<>(
                name,
                motor,
                encoder);
        return new LimitedVelocityServo<>(v, param.kMaxVelM_S(), param.kMaxAccelM_S2(), param.kMaxDecel());
    }

    


    public static VelocityServo<Distance100> neoVelocityServo(
            String name,
            int canId,
            boolean motorPhase,
            double gearRatio,
            double wheelDiameter) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                gearRatio,
                wheelDiameter);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                name,
                motor,
                wheelDiameter * Math.PI);
        return new OutboardVelocityServo<>(
                name,
                motor,
                encoder);
    }

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static PositionServo<Distance100> neoPositionServo(
            String name,
            int canId,
            boolean motorPhase,
            double gearRatio,
            double wheelDiameter,
            double maxVel,
            double maxAccel,
            PIDController pidController) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                gearRatio,
                wheelDiameter);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                name,
                motor,
                wheelDiameter * Math.PI);
        VelocityServo<Distance100> vServo = new OutboardVelocityServo<>(
                name,
                motor,
                encoder);
        return new PositionServo<>(
                name,
                vServo,
                encoder,
                maxVel,
                pidController,
                new TrapezoidProfile100(maxVel, maxAccel, 0.05),
                Distance100.instance);
    }

    public static PositionServo<Angle100> neoPositionServo(
            String name,
            int canId,
            boolean motorPhase,
            double gearRatio,
            double maxVel,
            double maxAccel,
            double kP) {
        NeoTurningMotor motor = new NeoTurningMotor(
                name,
                canId,
                motorPhase,
                gearRatio);
        NeoTurningEncoder encoder = new NeoTurningEncoder(
                name,
                motor);
        VelocityServo<Angle100> vServo = new OutboardVelocityServo<>(
                name,
                motor,
                encoder);
        return new PositionServo<>(
                name,
                vServo,
                encoder,
                maxVel,
                new PIDController(kP, 0, 0),
                new TrapezoidProfile100(maxVel, maxAccel, 0.05),
                Angle100.instance);
    }

    public static PositionServo<Angle100> neoPositionServo(
            String name,
            int canId,
            boolean motorPhase,
            SysParam param,
            PIDController controller) {
        NeoTurningMotor motor = new NeoTurningMotor(
                name,
                canId,
                motorPhase,
                param.kGearRatio());
        NeoTurningEncoder encoder = new NeoTurningEncoder(
                name,
                motor);
        VelocityServo<Angle100> vServo = new OutboardVelocityServo<>(
                name,
                motor,
                encoder);
        return new PositionServo<>(
                name,
                vServo,
                encoder,
                param.kMaxVelM_S(),
                controller,
                new TrapezoidProfile100(param.kMaxVelM_S(), param.kMaxAccelM_S2(), 0.05),
                Angle100.instance);
    }

    private ServoFactory() {
        //
    }
}

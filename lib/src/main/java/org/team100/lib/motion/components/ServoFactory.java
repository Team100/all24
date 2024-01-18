package org.team100.lib.motion.components;

import org.team100.lib.encoder.drive.NeoDriveEncoder;
import org.team100.lib.encoder.turning.NeoTurningEncoder;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;

public class ServoFactory {

    /**
     * Position control using velocity feedforward and proportional feedback.
     * Velocity control using outboard SparkMax controller.
     */
    public static PositionServo<Distance> neoPositionServo(
            String name,
            int canId,
            boolean motorPhase,
            double gearRatio,
            double wheelDiameter,
            double maxVel,
            double maxAccel,
            double kP) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                gearRatio,
                wheelDiameter);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                name + "/encoder",
                motor,
                wheelDiameter * Math.PI);
        VelocityServo<Distance> vServo = new OutboardVelocityServo<>(
                name + "/velocity",
                motor,
                encoder);
        return new PositionServo<>(
                name + "/position",
                vServo,
                encoder,
                maxVel,
                new PIDController(kP, 0, 0),
                new TrapezoidProfile100(maxVel, maxAccel, 0.05),
                Distance.instance);
    }
    public static PositionServo<Angle> neoPositionServo(
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
                name + "/encoder",
                motor);
        VelocityServo<Angle> vServo = new OutboardVelocityServo<>(
                name + "/velocity",
                motor,
                encoder);
        return new PositionServo<>(
                name + "/position",
                vServo,
                encoder,
                maxVel,
                new PIDController(kP, 0, 0),
                new TrapezoidProfile100(maxVel, maxAccel, 0.05),
                Angle.instance);
    }

    private ServoFactory() {
        //
    }
}

package org.team100.frc2024.motion.climber;

import org.team100.lib.encoder.drive.NeoDriveEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.SelectableVelocityServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private static final double kGearRatio = 20.0;
    private static final double kWinchDiameterM = 0.01;

    private final PositionServo<Distance> s1;
    private final PositionServo<Distance> s2;

    public ClimberSubsystem(String name1, String name2, int canID1, int canID2) {
        s1 = newPositionServo(name1, canID1, false);
        s2 = newPositionServo(name2, canID2, true);
    }

    /** Set velocity in meters per second */
    public void set(double value) {
        s1.setVelocity(value);
        s2.setVelocity(value);
    }

    public void setPosition(double value) {
        s1.setPosition(value);
        s2.setPosition(value);
    }

    @Override
    public void periodic() {
        s1.periodic();
        s2.periodic();
    }

    /**
     * Using the velocity servo means we can select onboard or outboard velocity
     * closed-loop, and control position with the position servo.
     */
    private static PositionServo<Distance> newPositionServo(
            String name,
            int canId,
            boolean motorPhase) {
        NeoDriveMotor motor = new NeoDriveMotor(
                name,
                canId,
                motorPhase,
                kGearRatio,
                kWinchDiameterM);
        NeoDriveEncoder encoder = new NeoDriveEncoder(
                name + "/encoder",
                motor,
                kWinchDiameterM * Math.PI);
        VelocityServo<Distance> vServo = new SelectableVelocityServo<>(
                name + "/velocity",
                motor,
                encoder,
                new PIDController(1, 0, 0),
                new SimpleMotorFeedforward(0, 1));
        return new PositionServo<>(
                name + "/position",
                vServo,
                encoder,
                1,
                new PIDController(1, 0, 0),
                new TrapezoidProfile100(1, 1, 0.05),
                Distance.instance);
    }
}

package org.team100.frc2024.motion.indexer;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.turning.NeoTurningEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.turning.NeoTurningMotor;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Angle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final TrapezoidProfile100 m_trapezoid;
    private State100 setpoint;
    private final PositionServo<Angle> m_servo;
    public IndexerSubsystem(String name, int canID) {
        m_servo = newPositionServo(name, canID, true);
        m_trapezoid = new TrapezoidProfile100(
                new Constraints100(4, 10), 0.05);
        setpoint = new State100(0, 0, 0);
    }
    @Override
    public void periodic() {
        m_servo.periodic();
        }
    public void set(double value) {
        setpoint = m_trapezoid.calculate(0.02, setpoint, new State100(value, 0, 0));
        m_servo.setVelocity(setpoint.x());
    }

    public void setPosition(double value) {
        m_servo.setPosition(value);
    }
    private static PositionServo<Angle> newPositionServo(
            String name,
            int canId,
            boolean motorPhase) {
        NeoTurningMotor motor = new NeoTurningMotor(
                name,
                canId,
                motorPhase);
        NeoTurningEncoder encoder = new NeoTurningEncoder(
                name + "/encoder",
                motor);
        VelocityServo<Angle> vServo = new VelocityServo<>(
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
                new ChoosableProfile(1, 1, ChoosableProfile.Mode.TRAPEZOID),
                Angle.instance);
    }
}

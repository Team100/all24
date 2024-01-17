package org.team100.frc2024.motion.intake;

import org.team100.lib.controller.State100;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.Constraints;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final NeoDriveMotor bottomRoller;
    private final NeoDriveMotor topRoller;
    private final TrapezoidProfile100 m_trapezoid;
    private State100 setpoint;

    public IntakeSubsystem(String name1, String name2, int canID1, int canID2) {
        topRoller = new NeoDriveMotor(name1, canID1, true, 2, 0.05);
        bottomRoller = new NeoDriveMotor(name2, canID2, false, 2, 0.05);
        m_trapezoid = new TrapezoidProfile100(
                new Constraints(2, 5), 0.05);
        setpoint = new State100(0, 0, 0);
    }

    public void set(double value) {
        setpoint = m_trapezoid.calculate(0.02, setpoint, new State100(value, 0, 0));
        topRoller.setVelocity(setpoint.x(), 0);
        bottomRoller.setVelocity(setpoint.x(), 0);
    }
}

package org.team100.frc2024.motion.shooter;

import org.team100.lib.controller.State100;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final NeoDriveMotor leftShooter;
    private final NeoDriveMotor rightShooter;
    private final TrapezoidProfile100 m_trapezoid;
    private State100 setpoint;

    public ShooterSubsystem(String name1, String name2, int canID1, int canID2) {
        leftShooter = new NeoDriveMotor(name1, canID1, true, 1, 0.1);
        rightShooter = new NeoDriveMotor(name2, canID2, false, 1, 0.1);
        m_trapezoid = new TrapezoidProfile100(
                new Constraints100(8, 30), 0.05);
        setpoint = new State100(0, 0, 0);
    }

    public void set(double value) {
        
        setpoint = m_trapezoid.calculate(0.02, setpoint, new State100(value, 0, 0));

        leftShooter.setVelocity(setpoint.x(), 0);
        rightShooter.setVelocity(setpoint.x(), 0);
    }
}

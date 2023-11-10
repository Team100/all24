package org.team100.rolly_grabber.subsystems.manipulator;

import org.team100.lib.commands.InitCommand;
import org.team100.lib.telemetry.Telemetry;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Rolly grabber subsystem that supports direction, current limiting, and
 * holding.
 * 
 * Note, this class has no motor temperature protections, so add some:
 * 
 * TODO: add motor temperature sensing.
 * TODO: add a "hold" timer.
 * TODO: add current sensing.
 */
public class Manipulator extends Subsystem implements ManipulatorInterface {
    private final Telemetry t = Telemetry.get();
    private final WPI_TalonSRX m_motor;

    // package-private for testing.
    Manipulator() {
        m_motor = new WPI_TalonSRX(10);
        m_motor.configFactoryDefault();
        m_motor.setSafetyEnabled(false);
        m_motor.enableCurrentLimit(true);
        m_motor.configContinuousCurrentLimit(0);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configPeakOutputForward(1);
        m_motor.configPeakOutputReverse(-1);
        m_motor.configPeakCurrentLimit(30);
        m_motor.configPeakCurrentDuration(1000);
    }

    // package-private for testing
    void set(double speed1_1, int currentLimit) {
        m_motor.configPeakCurrentLimit(currentLimit);
        m_motor.set(speed1_1);
        t.log("/Manipulator/Output Current amps", m_motor.getStatorCurrent());
        t.log("/Manipulator/Input Current amps", m_motor.getSupplyCurrent());
    }

    @Override
    public Command stop() {
        return cmd(0, 30);
    }

    @Override
    public Command intake() {
        return cmd(-0.8, 45);
    }

    @Override
    public Command hold() {
        return cmd(-0.2, 30);
    }

    @Override
    public Command eject() {
        return cmd(0.8, 30);
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }

    private Command cmd(double speed1_1, int currentLimit) {
        return new InitCommand(() -> set(speed1_1, currentLimit), this);
    }
}

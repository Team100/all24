package org.team100.frc2023.subsystems;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.Telemetry;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Manipulator extends Subsystem implements ManipulatorInterface {
    private static class Noop extends Subsystem implements ManipulatorInterface {

        @Override
        public Subsystem subsystem() {
            return this;
        }

        @Override
        public void set(double speed1_1, int currentLimit) {
        }

        @Override
        public double getStatorCurrent() {
            return 0;
        }

    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public ManipulatorInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                    return new Manipulator();
                default:
                    return new Noop();
            }
        }
    }

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonSRX m_motor;

    private Manipulator() {
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

    public void set(double speed1_1, int currentLimit) {
        m_motor.configPeakCurrentLimit(currentLimit);
        m_motor.set(speed1_1);
        t.log("/Manipulator/Output Current amps", m_motor.getStatorCurrent());
        t.log("/Manipulator/Input Current amps", m_motor.getSupplyCurrent());
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent();
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }
}

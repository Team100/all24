package org.team100.frc2024.motion.climber;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements Glassy {
    private static final int kCurrentLimit = 40;
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final CANSparkFlex s1;
    private final CANSparkFlex s2;

    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        switch (Identity.instance) {
            case COMP_BOT:
                s1 = new CANSparkFlex(leftClimberID, MotorType.kBrushless);
                s2 = new CANSparkFlex(rightClimberID, MotorType.kBrushless);
                s2.setInverted(false);
                s1.setInverted(true);
                s1.setSmartCurrentLimit(kCurrentLimit);
                s2.setSmartCurrentLimit(kCurrentLimit);
                break;
            case BLANK:
            default:
                s1 = new CANSparkFlex(60, MotorType.kBrushless);
                s2 = new CANSparkFlex(61, MotorType.kBrushless);
                s2.setInverted(true);
        }
    }

    public void setLeftWithSoftLimits(double value) {
        if (s1.getEncoder().getPosition() > 300 && value >= 0) {
            s1.set(0);
            return;
        }

        if (s1.getEncoder().getPosition() < 5 && value <= 0) {
            s1.set(0);
            return;
        }
        // s1.set(value);
        Telemetry.get().log(Level.DEBUG, m_name, "LEFT VALUE", value);
    }

    public void setRightWithSoftLimits(double value) {
        if (s2.getEncoder().getPosition() > 300 && value >= 0) {
            s2.set(0);
            return;
        }

        if (s2.getEncoder().getPosition() < 5 && value <= 0) {
            s2.set(0);
            return;
        }
        // s2.set(value);
        Telemetry.get().log(Level.DEBUG, m_name, "RIGHT VALUE", value);
    }

    public void zeroClimbers() {
        s1.getEncoder().setPosition(0);
        s2.getEncoder().setPosition(0);

    }

    public void setLeft(double value) {
        s1.set(value);
    }

    public void setRight(double value) {
        s2.set(value);
    }

    public double getRightPosition() {
        return s2.getEncoder().getPosition();
    }

    public double getLeftPosition() {
        return s1.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        t.log(Level.DEBUG, m_name, "CLIMBER 1 ENCODER", s1.getEncoder().getPosition());
        t.log(Level.DEBUG, m_name, "CLIMBER 2 ENCODER", s2.getEncoder().getPosition());
        t.log(Level.DEBUG, m_name, "current (A) CLIMVER 1", s1.getOutputCurrent());
        t.log(Level.DEBUG, m_name, "current (A) CLIMBER 2", s2.getOutputCurrent());
        t.log(Level.DEBUG, m_name, "DUTY CYCLE 1", s1.getAppliedOutput());
        t.log(Level.DEBUG, m_name, "DUTY CYCLE 2", s2.getAppliedOutput());
        t.log(Level.DEBUG, m_name, "RPM 1", s1.getEncoder().getVelocity());
        t.log(Level.DEBUG, m_name, "RPM 2", s2.getEncoder().getVelocity());
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

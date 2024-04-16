package org.team100.frc2024.motion.climber;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.duty_cycle.VortexEncoder;
import org.team100.lib.motor.duty_cycle.VortexProxy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements Glassy {
    private static final int kCurrentLimit = 40;
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final Motor100<Distance100> v1;
    private final Encoder100<Distance100> e1;
    private final Motor100<Distance100> v2;
    private final Encoder100<Distance100> e2;

    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        switch (Identity.instance) {
            case COMP_BOT:
                VortexProxy vp1 = new VortexProxy(m_name + "/left", leftClimberID, true, kCurrentLimit);
                e1 = new VortexEncoder( vp1);
                v1 = vp1;
                VortexProxy vp2 = new VortexProxy(m_name + "/right", rightClimberID, false, kCurrentLimit);
                e2 = new VortexEncoder( vp2);
                v2 = vp2;
                break;
            default:
                // for testing and simulation
                SimulatedMotor<Distance100> vs1 = new SimulatedMotor<>(m_name + "left", 1);
                e1 = new SimulatedEncoder<>(m_name + "left", vs1, 1, -Double.MAX_VALUE, Double.MAX_VALUE);
                v1 = vs1;
                SimulatedMotor<Distance100> vs2 = new SimulatedMotor<>(m_name + "right", 1);
                e2 = new SimulatedEncoder<>(m_name + "right", vs2, 1, -Double.MAX_VALUE, Double.MAX_VALUE);
                v2 = vs2;
        }
    }

    public void setLeftWithSoftLimits(double value) {
        if (e1.getPosition() > 300 && value >= 0) {
            v1.setDutyCycle(0);
            return;
        }

        if (e1.getPosition() < 5 && value <= 0) {
            v1.setDutyCycle(0);
            return;
        }
        // s1.set(value);
        Telemetry.get().log(Level.DEBUG, m_name, "LEFT VALUE", value);
    }

    public void setRightWithSoftLimits(double value) {
        if (e2.getPosition() > 300 && value >= 0) {
            v2.setDutyCycle(0);
            return;
        }

        if (e2.getPosition() < 5 && value <= 0) {
            v2.setDutyCycle(0);
            return;
        }
        // s2.set(value);
        Telemetry.get().log(Level.DEBUG, m_name, "RIGHT VALUE", value);
    }

    public void zeroClimbers() {
        e1.reset();
        e2.reset();

    }

    public void setLeft(double value) {
        v1.setDutyCycle(value);
    }

    public void setRight(double value) {
        v2.setDutyCycle(value);
    }

    public double getRightPosition() {
        return e2.getPosition();
    }

    public double getLeftPosition() {
        return e1.getPosition();
    }

    @Override
    public void periodic() {
        t.log(Level.DEBUG, m_name, "CLIMBER 1 ENCODER", e1.getPosition());
        t.log(Level.DEBUG, m_name, "CLIMBER 2 ENCODER", e2.getPosition());
        t.log(Level.DEBUG, m_name, "RPM 1", e1.getRate());
        t.log(Level.DEBUG, m_name, "RPM 2", e2.getRate());
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

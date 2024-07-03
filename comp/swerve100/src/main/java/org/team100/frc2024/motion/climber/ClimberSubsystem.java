package org.team100.frc2024.motion.climber;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.motor.MotorPhase;
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
    private final Telemetry.Logger t;
    private final String m_name;
    private final DutyCycleMotor100 v1;
    private final Encoder100<Distance100> e1;
    private final DutyCycleMotor100 v2;
    private final Encoder100<Distance100> e2;

    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        t = Telemetry.get().rootLogger(m_name);
        switch (Identity.instance) {
            case COMP_BOT:
                VortexProxy vp1 = new VortexProxy(
                        m_name + "/left",
                        t.child("/left"),
                        leftClimberID,
                        MotorPhase.FORWARD,
                        kCurrentLimit);
                e1 = new VortexEncoder(vp1);
                v1 = vp1;
                VortexProxy vp2 = new VortexProxy(
                        m_name + "/right",
                        t.child("/right"),
                        rightClimberID,
                        MotorPhase.REVERSE,
                        kCurrentLimit);
                e2 = new VortexEncoder(vp2);
                v2 = vp2;
                break;
            default:
                // for testing and simulation
                SimulatedMotor<Distance100> vs1 = new SimulatedMotor<>(
                        m_name + "left", t.child("left"), 1);
                e1 = new SimulatedEncoder<>(m_name + "left", t.child("left"),
                        vs1, 1, -Double.MAX_VALUE, Double.MAX_VALUE);
                v1 = vs1;
                SimulatedMotor<Distance100> vs2 = new SimulatedMotor<>(
                        m_name + "right", t.child("right"), 1);
                e2 = new SimulatedEncoder<>(m_name + "right", t.child("right"), vs2, 1, -Double.MAX_VALUE,
                        Double.MAX_VALUE);
                v2 = vs2;
        }
    }

    public void setLeftWithSoftLimits(double value) {
        OptionalDouble e1Position = e1.getPosition();
        if (e1Position.isEmpty()) {
            v1.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() > 300 && value >= 0) {
            v1.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() < 5 && value <= 0) {
            v1.setDutyCycle(0);
            return;
        }
        t.logDouble(Level.DEBUG, "LEFT VALUE", () -> value);
    }

    public void setRightWithSoftLimits(double value) {
        OptionalDouble e2Position = e2.getPosition();
        if (e2Position.isEmpty()) {
            v2.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() > 300 && value >= 0) {
            v2.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() < 5 && value <= 0) {
            v2.setDutyCycle(0);
            return;
        }
        t.logDouble(Level.DEBUG, "RIGHT VALUE", () -> value);
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

    public OptionalDouble getRightPosition() {
        return e2.getPosition();
    }

    public OptionalDouble getLeftPosition() {
        return e1.getPosition();
    }

    @Override
    public void periodic() {
        t.log(Level.DEBUG, "CLIMBER 1 ENCODER", e1.getPosition());
        t.log(Level.DEBUG, "CLIMBER 2 ENCODER", e2.getPosition());
        t.log(Level.DEBUG, "RPM 1", e1.getRate());
        t.log(Level.DEBUG, "RPM 2", e2.getRate());
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

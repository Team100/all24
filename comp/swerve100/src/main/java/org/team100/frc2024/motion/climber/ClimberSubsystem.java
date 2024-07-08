package org.team100.frc2024.motion.climber;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.IncrementalLinearEncoder;
import org.team100.lib.encoder.SimulatedLinearEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.motor.duty_cycle.VortexEncoder;
import org.team100.lib.motor.duty_cycle.VortexProxy;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements Glassy {
    private static final int kCurrentLimit = 40;
    // is this an 18 tooth 35-series sprocket?
    private static final double kSprocketDiameterM = 0.055;
    // TODO: is this the right reduction?
    private static final double kReduction = 16;
    private final Logger m_logger;
    private final BareMotor v1;
    private final IncrementalLinearEncoder e1;
    private final BareMotor v2;
    private final IncrementalLinearEncoder e2;

    public ClimberSubsystem(Logger parent, int leftClimberID, int rightClimberID) {
        m_logger = parent.child(this);
        switch (Identity.instance) {
            case COMP_BOT:
                VortexProxy vp1 = new VortexProxy(
                        m_logger.child("left"),
                        leftClimberID,
                        MotorPhase.FORWARD,
                        kCurrentLimit);
                e1 = new VortexEncoder(vp1);
                v1 = vp1;
                VortexProxy vp2 = new VortexProxy(
                        m_logger.child("right"),
                        rightClimberID,
                        MotorPhase.REVERSE,
                        kCurrentLimit);
                e2 = new VortexEncoder(vp2);
                v2 = vp2;
                break;
            default:
                // for testing and simulation
                SimulatedBareMotor vs1 = new SimulatedBareMotor(m_logger.child("left"), 1);
                LinearMechanism m1 = new LinearMechanism(vs1, kReduction, kSprocketDiameterM );
                e1 = new SimulatedLinearEncoder(m_logger.child("left"),
                        m1, -Double.MAX_VALUE, Double.MAX_VALUE);
                v1 = vs1;
                SimulatedBareMotor vs2 = new SimulatedBareMotor(m_logger.child("right"), 1);
                LinearMechanism m2 = new LinearMechanism(vs2, kReduction, kSprocketDiameterM );
                e2 = new SimulatedLinearEncoder(m_logger.child("right"), m2, -Double.MAX_VALUE,
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
        m_logger.logDouble(Level.TRACE, "LEFT VALUE", () -> value);
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
        m_logger.logDouble(Level.TRACE, "RIGHT VALUE", () -> value);
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
        m_logger.logOptionalDouble(Level.TRACE, "CLIMBER 1 ENCODER", e1::getPosition);
        m_logger.logOptionalDouble(Level.TRACE, "CLIMBER 2 ENCODER", e2::getPosition);
        m_logger.logOptionalDouble(Level.TRACE, "RPM 1", e1::getRate);
        m_logger.logOptionalDouble(Level.TRACE, "RPM 2", e2::getRate);
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

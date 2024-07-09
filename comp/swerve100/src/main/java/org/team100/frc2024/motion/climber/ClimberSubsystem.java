package org.team100.frc2024.motion.climber;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
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
    private final LinearMechanism m1;
    private final LinearMechanism m2;

    public ClimberSubsystem(Logger parent, int leftClimberID, int rightClimberID) {
        m_logger = parent.child(this);
        Logger leftLogger = m_logger.child("left");
        Logger rightLogger = m_logger.child("right");
        switch (Identity.instance) {
            case COMP_BOT:
                NeoVortexCANSparkMotor vp1 = new NeoVortexCANSparkMotor(
                        leftLogger,
                        leftClimberID,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        Feedforward100.makeNeoVortex(),
                        new PIDConstants(0, 0, 0));
                m1 = new LinearMechanism(vp1, new CANSparkEncoder(leftLogger, vp1), kReduction, kSprocketDiameterM);

                NeoVortexCANSparkMotor vp2 = new NeoVortexCANSparkMotor(
                        rightLogger,
                        rightClimberID,
                        MotorPhase.REVERSE,
                        kCurrentLimit,
                        Feedforward100.makeNeoVortex(),
                        new PIDConstants(0, 0, 0));
                m2 = new LinearMechanism(vp2, new CANSparkEncoder(rightLogger, vp2), kReduction, kSprocketDiameterM);
                break;
            default:
                // for testing and simulation
                SimulatedBareMotor vs1 = new SimulatedBareMotor(leftLogger, 1);
                m1 = new LinearMechanism(vs1, new SimulatedBareEncoder(leftLogger, vs1), kReduction,
                        kSprocketDiameterM);

                SimulatedBareMotor vs2 = new SimulatedBareMotor(rightLogger, 1);
                m2 = new LinearMechanism(vs2, new SimulatedBareEncoder(rightLogger, vs2), kReduction,
                        kSprocketDiameterM);
        }
    }

    public void setLeftWithSoftLimits(double value) {
        OptionalDouble e1Position = m1.getPositionM();
        if (e1Position.isEmpty()) {
            m1.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() > 300 && value >= 0) {
            m1.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() < 5 && value <= 0) {
            m1.setDutyCycle(0);
            return;
        }
        m_logger.logDouble(Level.TRACE, "LEFT VALUE", () -> value);
    }

    public void setRightWithSoftLimits(double value) {
        OptionalDouble e2Position = m2.getPositionM();
        if (e2Position.isEmpty()) {
            m2.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() > 300 && value >= 0) {
            m2.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() < 5 && value <= 0) {
            m2.setDutyCycle(0);
            return;
        }
        m_logger.logDouble(Level.TRACE, "RIGHT VALUE", () -> value);
    }

    public void zeroClimbers() {
        m1.resetEncoderPosition();
        m2.resetEncoderPosition();
    }

    public void setLeft(double value) {
        m1.setDutyCycle(value);
    }

    public void setRight(double value) {
        m2.setDutyCycle(value);
    }

    public OptionalDouble getRightPosition() {
        return m2.getPositionM();
    }

    public OptionalDouble getLeftPosition() {
        return m1.getPositionM();
    }

    @Override
    public void periodic() {
        m_logger.logOptionalDouble(Level.TRACE, "climber 1 position (m)", m1::getPositionM);
        m_logger.logOptionalDouble(Level.TRACE, "climber 2 position (m)", m2::getPositionM);
        m_logger.logOptionalDouble(Level.TRACE, "climber 1 velocity (m_s)", m1::getVelocityM_S);
        m_logger.logOptionalDouble(Level.TRACE, "climber 2 velocity (m_s)", m2::getVelocityM_S);
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

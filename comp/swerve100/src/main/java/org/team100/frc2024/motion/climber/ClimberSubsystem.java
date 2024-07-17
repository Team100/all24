package org.team100.frc2024.motion.climber;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.LinearMechanismInterface;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements Glassy {
    private static final double kMaxPositionM = 0.3;
    private static final double kUpPositionM = 0.28;
    private static final double kDownPositionM = 0.02;
    private static final double kMinPositionM = 0.01;
    /*******************************************
     * ALERT ALERT ALERT this current limit is uncalibrated.
     * 
     * The climber is rigged with ANSI #35 chain
     * 
     * https://www.peerchain.com/product/35r-roller-chain/
     * 
     * breaking strength is ~2500 lbs == ~11 kN.
     * 
     * Sprocket radius (below) is 0.022m, reduction is 45,
     * so breaking torque at the motor is 5.5 Nm, which is beyond the stall torque,
     * so it's not possible to break the chain with the motor.
     * 
     * In the past I think we broke the *hooks* which replaced the steel chain
     * plates with aluminum. The renewed design doubles the hook plates.
     * 
     * https://docs.revrobotics.com/brushless/neo/vortex
     * 
     * Lifting the 152-lb (max) robot with two climbers would take 676/2=338N
     * per side, which is only 0.165Nm, which is about 5% of the stall torque. The
     * stall current is 211 A, and 5% of that is about 10 A, so that's the "holding"
     * current.
     * 
     * To actually move the climber takes more than the holding current. Say we want
     * to move 0.2m with the same 338N gravity force, that would require about 68 J.
     * Say we wanted to do that in 1s, that would require 68 W. Assuming a bit of
     * sag, 68 W would require something like 7 amps.
     * 
     * There's also some friction.
     * 
     * A reasonable starting point would be 20: it shouldn't self destruct but it
     * should lift the robot.
     * 
     * TODO: hard stops, top and bottom.
     */
    private static final int kCurrentLimit = 20;
    private static final double kClimbingForceN = 676;
    private static final double kHomingForceN = 10;

    /**
     * 15 tooth 35-series sprocket
     * https://wcproducts.info/files/frc/drawings/Web-%2335%20Double%20Hub%20Sprockets.pdf
     */
    private static final double kSprocketDiameterM = 0.045;
    /**
     * one 5:1 and one 9:1 stage
     * https://www.revrobotics.com/rev-21-2103/
     * https://www.revrobotics.com/rev-21-2129/
     */
    private static final double kReduction = 45;

    private final SupplierLogger m_logger;
    private final LinearMechanismInterface m_left;
    private final LinearMechanismInterface m_right;

    public ClimberSubsystem(SupplierLogger parent, int leftClimberID, int rightClimberID) {
        m_logger = parent.child(this);
        Util.warn("\n**** Uncalibrated climber current limit!!!  FIX THIS FOR COMP! ****\n");
        Util.warn("\n**** Uncalibrated climber polarity!!!  FIX THIS FOR COMP! ****\n");
        Util.warn("\n**** Uncalibrated climber PID!!!  FIX THIS FOR COMP! ****\n");
        SupplierLogger leftLogger = m_logger.child("left");
        SupplierLogger rightLogger = m_logger.child("right");
        switch (Identity.instance) {
            case COMP_BOT -> {
                m_left = comp(leftLogger, leftClimberID, MotorPhase.REVERSE);
                m_right = comp(rightLogger, rightClimberID, MotorPhase.FORWARD);
            }
            default -> {
                m_left = simulated(leftLogger);
                m_right = simulated(rightLogger);
            }
        }
    }

    public Command upPosition(SupplierLogger logger) {
        return new ClimberPosition(logger, kUpPositionM, this);
    }

    public Command downPosition(SupplierLogger logger) {
        return new ClimberPosition(logger, kDownPositionM, this);
    }

    private static LinearMechanismInterface comp(SupplierLogger logger, int id, MotorPhase phase) {
        NeoVortexCANSparkMotor vp2 = new NeoVortexCANSparkMotor(
                logger,
                id,
                phase,
                kCurrentLimit,
                Feedforward100.makeNeoVortex(),
                new PIDConstants(0, 0, 0));
        return new LinearMechanism(vp2,
                new CANSparkEncoder(logger, vp2),
                kReduction,
                kSprocketDiameterM);
    }

    /**
     * For testing and simulation.
     * Neo vortex free speed is 6784 rpm, 710 rad/s
     * 
     * https://docs.revrobotics.com/brushless/neo/vortex
     */
    private static LinearMechanismInterface simulated(SupplierLogger logger) {
        SimulatedBareMotor vs2 = new SimulatedBareMotor(logger, 710);
        return new LinearMechanism(vs2,
                new SimulatedBareEncoder(logger, vs2),
                kReduction,
                kSprocketDiameterM);
    }

    /** Use a low, safe force limit for homing. */
    public void setHomingForce() {
        m_left.setForceLimit(kHomingForceN);
        m_right.setForceLimit(kHomingForceN);
    }

    /** Use a force sufficient to climb. */
    public void setClimbingForce() {
        m_left.setForceLimit(kClimbingForceN);
        m_right.setForceLimit(kClimbingForceN);
    }

    public void setLeftWithSoftLimits(double value) {
        OptionalDouble e1Position = m_left.getPositionM();
        if (e1Position.isEmpty()) {
            m_left.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() > 300 && value >= 0) {
            m_left.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() < 5 && value <= 0) {
            m_left.setDutyCycle(0);
            return;
        }
        m_logger.logDouble(Level.TRACE, "LEFT VALUE", () -> value);
    }

    public void setRightWithSoftLimits(double value) {
        OptionalDouble e2Position = m_right.getPositionM();
        if (e2Position.isEmpty()) {
            m_right.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() > 300 && value >= 0) {
            m_right.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() < 5 && value <= 0) {
            m_right.setDutyCycle(0);
            return;
        }
        m_logger.logDouble(Level.TRACE, "RIGHT VALUE", () -> value);
    }

    public void zeroClimbers() {
        m_left.resetEncoderPosition();
        m_right.resetEncoderPosition();
    }

    public void zeroLeft() {
        m_left.resetEncoderPosition();
    }

    public void zeroRight() {
        m_right.resetEncoderPosition();
    }

    /** Sets left climb duty cycle */
    public void setLeft(double value) {
        m_left.setDutyCycle(value);
    }

    /** Sets right climb duty cycle */
    public void setRight(double value) {
        m_right.setDutyCycle(value);
    }

    public void setLeftVelocityM_S(double v) {
        m_left.setVelocity(v, 0, 0);
    }

    public void setRightVelocityM_S(double v) {
        m_right.setVelocity(v, 0, 0);
    }

    public OptionalDouble getLeftPositionM() {
        return m_left.getPositionM();
    }

    public OptionalDouble getRightPositionM() {
        return m_right.getPositionM();
    }

    public OptionalDouble getLeftVelocity() {
        return m_left.getVelocityM_S();
    }

    public OptionalDouble getRightVelocity() {
        return m_right.getVelocityM_S();
    }

    public LinearMechanismInterface getLeft() {
        return m_left;
    }

    public LinearMechanismInterface getRight() {
        return m_right;
    }

    @Override
    public void periodic() {
        m_logger.logOptionalDouble(Level.TRACE, "left position (m)", m_left::getPositionM);
        m_logger.logOptionalDouble(Level.TRACE, "right position (m)", m_right::getPositionM);
        m_logger.logOptionalDouble(Level.TRACE, "left velocity (m_s)", m_left::getVelocityM_S);
        m_logger.logOptionalDouble(Level.TRACE, "right velocity (m_s)", m_right::getVelocityM_S);
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

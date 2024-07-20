package org.team100.frc2024.motion.climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motion.LimitedLinearMechanism;
import org.team100.lib.motion.SimpleLinearMechanism;
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

    /** This will break the mechanism if you hit the hard stop. */
    private static final double kClimbingForceN = 676;

    /** This won't break anything. */
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
    private final LimitedLinearMechanism m_left;
    private final LimitedLinearMechanism m_right;

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

    private static LimitedLinearMechanism comp(SupplierLogger logger, int id, MotorPhase phase) {
        Feedforward100 ff = Feedforward100.makeNeoVortex();
        /** The PID constants units are duty cycle per RPM, so very small numbers. */
        PIDConstants pid = new PIDConstants(0, 0, 0);
        NeoVortexCANSparkMotor motor = new NeoVortexCANSparkMotor(
                logger,
                id,
                phase,
                kCurrentLimit,
                ff,
                pid);
        SimpleLinearMechanism mech = new SimpleLinearMechanism(
                motor,
                new CANSparkEncoder(logger, motor),
                kReduction,
                kSprocketDiameterM);
        return new LimitedLinearMechanism(mech, kMinPositionM, kMaxPositionM);
    }

    /**
     * For testing and simulation.
     * Neo vortex free speed is 6784 rpm, 710 rad/s
     * 
     * https://docs.revrobotics.com/brushless/neo/vortex
     */
    private static LimitedLinearMechanism simulated(SupplierLogger logger) {
        SimulatedBareMotor vs2 = new SimulatedBareMotor(logger, 710);
        SimpleLinearMechanism lm = new SimpleLinearMechanism(
                vs2,
                new SimulatedBareEncoder(logger, vs2),
                kReduction,
                kSprocketDiameterM);
        return new LimitedLinearMechanism(lm, kMinPositionM, kMaxPositionM);
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

    public LimitedLinearMechanism getLeft() {
        return m_left;
    }

    public LimitedLinearMechanism getRight() {
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

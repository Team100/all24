package org.team100.frc2024.motion.climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.LimitedLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.LinearPositionServo;
import org.team100.lib.motion.servo.OnboardLinearDutyCyclePositionServo;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
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

    private final LimitedLinearMechanism m_left;
    private final LimitedLinearMechanism m_right;

    private final LinearPositionServo m_leftServo;
    private final LinearPositionServo m_rightServo;

    // LOGGERS
    private final OptionalDoubleLogger m_log_left_position;
    private final OptionalDoubleLogger m_log_right_position;
    private final OptionalDoubleLogger m_log_left_velocity;
    private final OptionalDoubleLogger m_log_right_velocity;

    public ClimberSubsystem(LoggerFactory parent, int leftClimberID, int rightClimberID) {
        LoggerFactory child = parent.child(this);
        m_log_left_position = child.optionalDoubleLogger(Level.TRACE, "left position (m)");
        m_log_right_position = child.optionalDoubleLogger(Level.TRACE, "right position (m)");
        m_log_left_velocity = child.optionalDoubleLogger(Level.TRACE, "left velocity (m_s)");
        m_log_right_velocity = child.optionalDoubleLogger(Level.TRACE, "right velocity (m_s)");

        Util.warn("**** Uncalibrated climber current limit!!!  FIX THIS FOR COMP! ****");
        Util.warn("**** Uncalibrated climber polarity!!!  FIX THIS FOR COMP! ****");
        Util.warn("**** Uncalibrated climber PID!!!  FIX THIS FOR COMP! ****");
        LoggerFactory leftLogger = child.child("left");
        LoggerFactory rightLogger = child.child("right");
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
        m_leftServo = new OnboardLinearDutyCyclePositionServo(
                child.child("left"),
                m_left,
                new PIDController(0.1, 0, 0),
                new TrapezoidProfile100(0.02, 0.1, 0.01));
        m_rightServo = new OnboardLinearDutyCyclePositionServo(
                child.child("right"),
                m_right,
                new PIDController(0.1, 0, 0),
                new TrapezoidProfile100(0.02, 0.1, 0.02));
    }

    public Command upPosition() {
        return new ClimberPosition(kUpPositionM, this);
    }

    public Command downPosition() {
        return new ClimberPosition(kDownPositionM, this);
    }

    private static LimitedLinearMechanism comp(LoggerFactory logger, int id, MotorPhase phase) {
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
    private static LimitedLinearMechanism simulated(LoggerFactory logger) {
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

    public void setPosition(double goalM) {
        m_leftServo.setPosition(goalM, 0);
        m_rightServo.setPosition(goalM, 0);
    }

    public void stop() {
        m_leftServo.stop();
        m_rightServo.stop();
    }

    public void resetServos() {
        m_leftServo.reset();
        m_rightServo.reset();
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
        m_log_left_position.log(m_left::getPositionM);
        m_log_right_position.log(m_right::getPositionM);
        m_log_left_velocity.log(m_left::getVelocityM_S);
        m_log_right_velocity.log(m_right::getVelocityM_S);
    }
}

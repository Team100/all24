package org.team100.frc2024.motion.indexer;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.servo.LimitedLinearVelocityServo;
import org.team100.lib.motion.servo.ServoFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct-drive roller indexer.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.05m => 0.15 m/turn
 * therefore top speed is around 15 m/s.
 * 
 * This system has low intertia but a lot of friction,
 * and it's fragile. we want to eject as fast as possible
 * though, so try a high accel limit.
 */
public class IndexerSubsystem extends SubsystemBase implements Glassy {
    private static final double kMaxDecel = -50;
    private static final double kMaxAccel = 50;
    private static final double kMaxVelocity = 15;
    private static final double kGearRatio = 12.0;
    private static final double kWheelDiameterM = 0.05;
    private static final int kCurrentLimit = 30;

    /**
     * Surface velocity of whatever is turning in the indexer.
     */
    private static final double kIndexerVelocityM_S = 5;

    private final LimitedLinearVelocityServo m_servo;
    private final PIDConstants m_velocityConstants;
    private final Feedforward100 m_lowLevelFeedforwardConstants;

    DigitalInput beamBreak1;
    // DigitalInput beamBreak2;

    public IndexerSubsystem(LoggerFactory parent, int driveID) {
        LoggerFactory child = parent.child(this);
        m_velocityConstants = new PIDConstants(0.0001, 0, 0);
        m_lowLevelFeedforwardConstants = Feedforward100.makeNeo();

        switch (Identity.instance) {
            case COMP_BOT:

                // beamBreak1 = new DigitalInput(4);
                // beamBreak2 = new DigitalInput(8);

                m_servo = ServoFactory.limitedNeoVelocityServo(
                        child,
                        driveID,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        kGearRatio,
                        kWheelDiameterM,
                        kMaxVelocity,
                        kMaxAccel,
                        kMaxDecel,
                        m_lowLevelFeedforwardConstants,
                        m_velocityConstants);
                break;
            case BLANK:
            default:
                m_servo = ServoFactory.limitedSimulatedVelocityServo(
                        child,
                        kGearRatio,
                        kWheelDiameterM,
                        kMaxVelocity,
                        kMaxAccel,
                        kMaxDecel);
        }
    }

    public void index() {
        m_servo.setVelocityM_S(kIndexerVelocityM_S);
    }

    public void indexWithBeamBreak() {
        if (beamBreak1.get()) {
            m_servo.setVelocityM_S(0);
        } else {
            m_servo.setVelocityM_S(kIndexerVelocityM_S);

        }
    }

    public void outdex() {
        m_servo.setVelocityM_S(-kIndexerVelocityM_S);
    }

    public void stop() {
        m_servo.stop();
    }

    public OptionalDouble getVelocity() {
        return m_servo.getVelocity();
    }
}

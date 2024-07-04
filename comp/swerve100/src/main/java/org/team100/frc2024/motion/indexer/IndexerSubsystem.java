package org.team100.frc2024.motion.indexer;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Distance100;

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
    private static final int kCurrentLimit = 30;

    /**
     * Surface velocity of whatever is turning in the indexer.
     */
    private static final double kIndexerVelocityM_S = 5;
    private final Logger m_logger;
    private final LimitedVelocityServo<Distance100> m_servo;
    private final PIDConstants m_velocityConstants;
    private final Feedforward100 m_lowLevelFeedforwardConstants;

    DigitalInput beamBreak1;
    // DigitalInput beamBreak2;

    public IndexerSubsystem(Logger parent, int driveID) {
        m_logger = parent.child(this);
        m_velocityConstants = new PIDConstants(0.0001, 0, 0);
        m_lowLevelFeedforwardConstants = Feedforward100.makeNeo();

        SysParam params = SysParam.limitedNeoVelocityServoSystem(
            12.0,
             0.05,
             15,
             50,
             -50);
        switch (Identity.instance) {
            case COMP_BOT:

                // beamBreak1 = new DigitalInput(4);
                // beamBreak2 = new DigitalInput(8);


                m_servo = ServoFactory.limitedNeoVelocityServo(
                        m_logger,
                        driveID,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        params,
                        m_lowLevelFeedforwardConstants,
                        m_velocityConstants);
                break;
            case BLANK:
            default:
                m_servo = ServoFactory.limitedSimulatedVelocityServo(
                        m_logger,
                        params);
        }
    }

    public void index() {
        m_servo.setVelocity(kIndexerVelocityM_S);
    }

    public void indexWithBeamBreak() {
        if(beamBreak1.get()){
            m_servo.setVelocity(0);
        } else {
            m_servo.setVelocity(kIndexerVelocityM_S);

        }
    }

    public void outdex() {
        m_servo.setVelocity(-kIndexerVelocityM_S);
    }

    public void stop() {
        m_servo.stop();
    }
    
    public OptionalDouble getVelocity() {
        return m_servo.getVelocity();
    }

    @Override
    public String getGlassName() {
        return "Indexer";
    }
}

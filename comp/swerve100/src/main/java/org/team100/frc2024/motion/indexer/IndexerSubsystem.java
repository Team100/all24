package org.team100.frc2024.motion.indexer;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

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
public class IndexerSubsystem extends SubsystemBase implements Speeding {
    // TODO: tune the current limit
    private static final int kCurrentLimit = 30;

    /**
     * Surface velocity of whatever is turning in the indexer.
     */
    private static final double kIndexerVelocityM_S = 3;
    private final String m_name;
    private final LimitedVelocityServo<Distance100> m_servo;
    private final SpeedingVisualization m_viz;

    public IndexerSubsystem(int driveID) {
        m_name = Names.name(this);
        SysParam params = SysParam.limitedNeoVelocityServoSystem(
            1.0,
             0.05,
             15,
             50,
             -50);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                m_servo = ServoFactory.limitedNeoVelocityServo(
                        m_name,
                        driveID,
                        true,
                        kCurrentLimit,
                        params);
                break;
            case BLANK:
            default:
                m_servo = ServoFactory.limitedSimulatedVelocityServo(
                        m_name,
                        params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    public void forward() {
        m_servo.setVelocity(kIndexerVelocityM_S);
    }

    public void stop() {
        m_servo.stop();
    }
    
    @Override
    public double getVelocity() {
        return m_servo.getVelocity();
    }

    @Override
    public void periodic() {
        m_servo.periodic();
        m_viz.periodic();
    }
}

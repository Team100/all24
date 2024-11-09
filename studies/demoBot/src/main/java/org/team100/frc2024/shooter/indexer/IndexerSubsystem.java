package org.team100.frc2024.shooter.indexer;

import java.util.OptionalDouble;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase implements Indexer {

    private final double kIndexerVelocityM_S;
    private final double m_objectLength;

    private final LoggerFactory m_logger;
    private final OutboardLinearPositionServo m_indexer;
    private final LinearMechanism m_linearMechanism;

    
    public IndexerSubsystem(LoggerFactory parent, LinearMechanism linearMechanism, double maxAccel, double objectLengthM, double indexVelocityM_S) {
        m_logger = parent.child(this);
        m_objectLength = objectLengthM;
        kIndexerVelocityM_S = indexVelocityM_S;
        m_linearMechanism = linearMechanism;
        m_indexer = new OutboardLinearPositionServo(m_logger, linearMechanism, new TrapezoidProfile100(indexVelocityM_S, maxAccel, 0.02));
    }

    public void index() {
        set(kIndexerVelocityM_S);
    }

    public void unindex() {
        set(-1.0 * kIndexerVelocityM_S);
    }
    
    public void indexOne() {
        setAngle(m_indexer.getPosition().getAsDouble() + m_objectLength);
    }

    public void unindexOne() {
        setAngle(m_indexer.getPosition().getAsDouble() - m_objectLength);
    }

    public void stop() {
        m_indexer.stop();
    }

    public OptionalDouble getVelocity() {
        return m_indexer.getVelocity();
    }

    @Override
    public String getGlassName() {
        return "Indexer";
    }

    @Override
    public void set(double value) {
        m_linearMechanism.setVelocity(value,0, 0);
    }

    public void setAngle(double value) {
        m_indexer.setPosition(value,0);
    }

    public double getAngle() {
        return m_indexer.getPosition().getAsDouble();
    }
}

package org.team100.frc2024.shooter.indexer;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.components.OutboardLinearPositionServo;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;

public class IndexerSubsystem implements Glassy {

    private final double kIndexerVelocityM_S;
    private final double m_objectLength;

    private final SupplierLogger m_logger;
    private final OutboardLinearPositionServo m_indexer;

    
    public IndexerSubsystem(SupplierLogger parent, LinearMechanism linearMechanism, double maxAccel, double objectLengthM, double indexVelocityM_S) {
        m_logger = parent.child(this);
        m_objectLength = objectLengthM;
        kIndexerVelocityM_S = indexVelocityM_S;
        m_indexer = new OutboardLinearPositionServo(m_logger, linearMechanism, new TrapezoidProfile100(indexVelocityM_S, maxAccel, 0.02));
    }

    public void index() {
        m_indexer.setVelocity(kIndexerVelocityM_S, 0);
    }

    public void unindex() {
        m_indexer.setVelocity(-1.0 * kIndexerVelocityM_S, 0);
    }
    
    public void indexOne() {
        m_indexer.setPosition(m_indexer.getPosition().getAsDouble() + m_objectLength,0);
    }

    public void unindexOne() {
        m_indexer.setPosition(m_indexer.getPosition().getAsDouble() - m_objectLength,0);
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
}

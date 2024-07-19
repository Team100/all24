package org.team100.frc2024.shooter;

import org.team100.frc2024.shooter.indexer.IndexerSubsystem;
import org.team100.frc2024.shooter.pivot.GravityServo;
import org.team100.frc2024.shooter.drumShooter.DrumShooter;
import org.team100.lib.commands.Subsystem100;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

public class PivotShooter extends Subsystem100{
    
    private final DrumShooter m_shooter;
    private final SupplierLogger m_logger;
    private final GravityServo m_pivot;
    private final IndexerSubsystem m_indexerSubsystem;

    private boolean atVelocity;

    private static final double shooterVelocity = 30;

    public PivotShooter(
            SupplierLogger parent,
            ShooterCollection shooterCollection) {
        m_logger = parent.child(this);
        // 50*Math.PI*kIndexWheelDiameterM,80*(Math.PI)*kIndexWheelDiameterM,0.02
        m_indexerSubsystem = new IndexerSubsystem(m_logger, shooterCollection.getIndexer(), shooterCollection.getIndexAccelM_S2(),shooterCollection.getIndexerDiameterM()*Math.PI/4,0.1);
        LinearVelocityServo[] shooter = shooterCollection.getShooters();
        m_shooter = new DrumShooter(parent, shooter[0], shooter[1]);
        m_pivot = shooterCollection.getPivot();
    }

    public void setAngle(double goalRad) {
        m_pivot.setPosition(goalRad);
    }

    public void setAngleMoving(double goalRad_S, double goalRad_S2) {
        m_pivot.setPosition(goalRad_S, goalRad_S2);
    }

    public void setAngleVelocity(double goalRad_S2) {
        double goalPercent = goalRad_S2 / (Math.PI*2*39);
        m_pivot.setVelocity(goalPercent);
    }

    public void spinUpShooter() {
        m_shooter.set(shooterVelocity);
    }

    public void spinUpShooterWithSpin() {
        m_shooter.setIndividual(shooterVelocity, shooterVelocity-10);
    }

    public void shootOne() {
        if (atVelocity) {
            m_indexerSubsystem.indexOne();
        }
    }

    public void shootAll() {
        if (atVelocity) {
            m_indexerSubsystem.index();
        }
    }

    public void idle() {
        m_shooter.set(0);
    }

    @Override
    public void periodic100(double dt) {
        atVelocity = m_shooter.atVeloctity();
        m_logger.logBoolean(Level.TRACE, "At velocity", () -> atVelocity);
    }

    @Override
    public String getGlassName() {
        return "Pivoted Shooter";
    }
}

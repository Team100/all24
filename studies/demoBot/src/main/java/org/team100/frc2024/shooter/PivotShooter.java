package org.team100.frc2024.shooter;

import org.team100.frc2024.shooter.drumShooter.DrumShooter;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.GravityServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotShooter extends SubsystemBase implements Glassy{
    
    private final DrumShooter m_shooter;
    private final SupplierLogger m_logger;
    private final GravityServo m_pivot;
    private final Servo m_indexerSubsystem;

    private boolean atVelocity;

    private static final double shooterVelocity = 30;

    public PivotShooter(
            SupplierLogger parent,
            ShooterCollection shooterCollection) {
        m_logger = parent.child(this);
        m_indexerSubsystem = shooterCollection.getIndexer();
        LinearVelocityServo[] shooter = shooterCollection.getShooters();
        m_shooter = new DrumShooter(m_logger, shooter[0], shooter[1]);
        m_pivot = shooterCollection.getPivot();
    }

    public void setAngle(double goalRad) {
        m_pivot.setPosition(goalRad);
    }

    public void setAngleVelocity(double goalRad_S2) {
        m_pivot.setVelocity(goalRad_S2);
    }

    public void spinUpShooter() {
        m_shooter.set(shooterVelocity);
    }

    public void spinUpShooterWithSpin() {
        m_shooter.setIndividual(shooterVelocity, shooterVelocity-10);
    }

    public void shootOne() {
        if (atVelocity) {
            m_indexerSubsystem.setAngle(m_indexerSubsystem.get()*360 + 90);
        }
    }

    public void shootAll() {
        if (atVelocity) {
            m_indexerSubsystem.setSpeed(1);
        }
    }

    public void idle() {
        m_shooter.set(0);
    }

    @Override
    public void periodic() {
        atVelocity = m_shooter.atVeloctity();
        m_logger.logBoolean(Level.TRACE, "At velocity", () -> atVelocity);
    }

    @Override
    public String getGlassName() {
        return "Pivoted Shooter";
    }
}

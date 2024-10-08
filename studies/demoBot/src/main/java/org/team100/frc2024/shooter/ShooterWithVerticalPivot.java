package org.team100.frc2024.shooter;

import org.team100.frc2024.shooter.drumShooter.DrumShooter;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardGravityServo;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWithVerticalPivot extends SubsystemBase implements Glassy{
    
    private final DrumShooter m_shooter;
    private final BooleanLogger m_logger;
    private final OutboardGravityServo m_pivot;
    private final Servo m_indexer;

    private boolean atVelocity;

    private static final double shooterVelocity = 30;

    public ShooterWithVerticalPivot(
            LoggerFactory parent,
            ShooterCollection shooterCollection) {
        LoggerFactory logger = parent.child(this);
        m_logger = logger.booleanLogger(Level.TRACE, "At velocity");
        m_indexer = shooterCollection.getIndexer();
        LinearVelocityServo[] shooter = shooterCollection.getShooters();
        m_shooter = new DrumShooter(logger, shooter[0], shooter[1]);
        m_pivot = shooterCollection.getPivot();
    }

    public void setAngle(double goalRad) {
        m_pivot.setPosition(goalRad);
    }

    public void spinUpShooter() {
        m_shooter.set(shooterVelocity);
    }

    public void spinUpShooterWithSpin() {
        m_shooter.setIndividual(shooterVelocity, shooterVelocity-10);
    }

    public void shootOne() {
        if (atVelocity) {
            m_indexer.setAngle(m_indexer.get()*360 + 90);
        }
    }

    //TODO fix this once testing with real shooter is complete
    public void shootAll() {
        if (atVelocity) {
            m_indexer.setSpeed(1);
        }
    }

    public void idle() {
        m_shooter.set(0);
    }

    @Override
    public void periodic() {
        atVelocity = m_shooter.atVeloctity();
        m_logger.log(() -> atVelocity);
    }

    @Override
    public String getGlassName() {
        return "Pivoted Shooter";
    }
}

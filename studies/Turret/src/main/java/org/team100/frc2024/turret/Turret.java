package org.team100.frc2024.turret;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase implements Glassy {
    private final OutboardAngularPositionServo m_motor;
    private final DoubleLogger m_goalPosition;
    private final DoubleLogger m_goalVelocity;
    public Turret(LoggerFactory parent, TurretCollection turretCollection) {
        LoggerFactory logger = parent.child(this);
        m_goalPosition = logger.doubleLogger(Level.TRACE, "Goal Position (rad)");
        m_goalVelocity = logger.doubleLogger(Level.TRACE, "Goal Velocity (rad_s)");
        m_motor = turretCollection.getTurret();
    }

    public void setAngle(double angle) {
        m_motor.setPosition(angle, 0);
        m_goalPosition.log(() -> angle);
    }

    public void setAngleWithVelocity(double angle, double angle_2) {
        m_motor.setPositionWithVelocity(angle, angle_2, 0);
        m_goalPosition.log(() -> angle);
        m_goalVelocity.log(() -> angle_2);
    }

    public void stop() {
        m_motor.stop();
    }
}

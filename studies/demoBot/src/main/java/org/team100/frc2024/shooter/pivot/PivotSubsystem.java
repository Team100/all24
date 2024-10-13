package org.team100.frc2024.shooter.pivot;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.servo.OutboardGravityServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase implements Glassy {
    
    private final OutboardGravityServo m_pivot;
    private final OptionalDoubleLogger m_logger;

    public PivotSubsystem(
            LoggerFactory parent,
            PivotCollection pivotCollection) {
        LoggerFactory logger = parent.child(this);
        m_logger = logger.optionalDoubleLogger(Level.TRACE, "Pivot Position (rad)");
        m_pivot = pivotCollection.getPivot();
    }

    public void setAngleRad(double angle) {
        m_pivot.setPosition(angle);
    }

    public OptionalDouble getAngleRad() {
        return m_pivot.getPositionRad();
    }

    public void setEncoderPosition(double positionRad) {
        m_pivot.setEncoderPosition(positionRad);
    }

    public void setTorqueLimit(double value) {
        m_pivot.setTorqueLimit(value);
    }

    public void stop() {
        m_pivot.stop();
    }

    @Override
    public void periodic() {
        m_logger.log(() -> getAngleRad());
    }

    @Override
    public String getGlassName() {
        return "Pivot";
    }
}

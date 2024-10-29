package org.team100.frc2024.shooter.pivot;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.Neo550CANSparkMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase implements Glassy {
    
    private final Neo550CANSparkMotor m_pivot;
    private final DoubleLogger m_logger;
    public PivotSubsystem(
            LoggerFactory parent,
            PivotCollection pivotCollection) {
        LoggerFactory logger = parent.child(this);
        m_logger = logger.doubleLogger(Level.TRACE, "Pivot Position (rad)");
        m_pivot = pivotCollection.getPivot();
    }

    // public void setAngleRad(double angle) {
    //     m_pivot.setD(angle);
    // }

    public void dutyCycle(double set) {
        m_pivot.setDutyCycle(set);
    }

    // public void setVelocityRad_S(double angle_S) {
    //     m_pivot.setVelocity(angle_S);
    // }

    public double getAngleRad() {
        return m_pivot.getPositionRot();
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

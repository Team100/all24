package org.team100.frc2024.shooter.indexer;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerServo extends SubsystemBase implements Indexer {
    
    private final Servo m_servo;
    private final DoubleLogger m_doubleLogger;

    private double speed;

    public IndexerServo(LoggerFactory parent, int channel) {
        LoggerFactory logger = parent.child(this);
        m_doubleLogger = logger.doubleLogger(Level.TRACE, "Angle (deg)");
        m_servo = new Servo(channel);
        stop();
    }

    public void set(double value) {
        speed = value;
    } 

    public void stop() {
        speed = 0;
    } 

    public void setAngle(double value) {
        stop();
        m_servo.setAngle(value);
    } 

    public double getAngle() {
        return m_servo.getAngle();
    }

    @Override
    public void periodic() {
        m_doubleLogger.log(() -> getAngle());
        m_servo.setAngle(getAngle() + speed);
    }

    @Override
    public String getGlassName() {
        return "Indexer";
    }
}

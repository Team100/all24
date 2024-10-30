package org.team100.frc2024.shooter.indexer;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.MotorPhase;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerServo extends SubsystemBase implements Indexer {
    
    private final PWM m_servo;
    private final DoubleLogger m_doubleLogger;
    private final int m_motorPhase;

    public IndexerServo(LoggerFactory parent, MotorPhase motorPhase,int channel) {
        LoggerFactory logger = parent.child(this);
        if (MotorPhase.FORWARD.equals(motorPhase)) {
            m_motorPhase = 1;
        } else {
            m_motorPhase = -1;
        }
        m_doubleLogger = logger.doubleLogger(Level.TRACE, "Angle (deg)");
        m_servo = new PWM(channel);
        stop();
    }

    @Override
    public void set(double value) {
        m_servo.setSpeed(value * m_motorPhase);
        m_doubleLogger.log(() -> value);
    } 

    @Override
    public void stop() {
        m_servo.setSpeed(0);
    } 

    @Override
    public String getGlassName() {
        return "Indexer";
    }
}

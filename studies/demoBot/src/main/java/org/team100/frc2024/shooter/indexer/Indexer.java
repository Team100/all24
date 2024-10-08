package org.team100.frc2024.shooter.indexer;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase implements Glassy {
    
    private final Servo m_servo;
    private double speed;

    public Indexer(int channel) {
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
        m_servo.setAngle(getAngle() + speed);
    }

    @Override
    public String getGlassName() {
        return "Indexer";
    }
}

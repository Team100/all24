package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is an example subsystem using the 1d components. */
public class Subsystem1d extends Subsystem {
    private final VelocityServo1d m_servo;
    DoubleSupplier m_supplier;

    public Subsystem1d() {
        m_servo = new VelocityServo1d();
    }

    public void setSupplier(DoubleSupplier supplier) {
        m_supplier = supplier;
    }

    public double getPositionM() {
        return 0.0; // actually position
    }

    @Override
    public void periodic() {
        if (m_supplier == null) {
            m_servo.setVelocityM_S(0);
            return;
        }
        m_servo.setVelocityM_S(m_supplier.getAsDouble());
    }
    
}

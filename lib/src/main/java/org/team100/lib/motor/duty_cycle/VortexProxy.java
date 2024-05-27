package org.team100.lib.motor.duty_cycle;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/** Very simple wrapper around SparkFlex for testing. */
public class VortexProxy implements Motor100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final CANSparkFlex m_motor;
    private final RelativeEncoder m_encoder;

    public VortexProxy(
            String name,
            int canId,
            boolean inverted,
            int currentLimit) {
        m_name = Names.append(name, this);
        m_motor = new CANSparkFlex(canId, MotorType.kBrushless);
        m_motor.setInverted(inverted);
        m_motor.setSmartCurrentLimit(currentLimit);
        m_encoder = m_motor.getEncoder();
    }

    private void set(double speed) {
        m_motor.set(speed);
        t.log(Level.DEBUG, m_name, "current (A)", m_motor.getOutputCurrent());
        t.log(Level.DEBUG, m_name, "duty cycle", m_motor.getAppliedOutput());
    }

    @Override
    public void setDutyCycle(double output) {
        set(output);
    }

    @Override
    public void setVelocity(double velocity, double accel) {
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public void setVelocity(double velocity, double accel, double torque) {
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public double getTorque() {
        throw new UnsupportedOperationException("Unimplemented method 'getTorque'");
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    double getVelocity() {
        return m_encoder.getVelocity();
    }

    double getPosition() {
        return m_encoder.getPosition();
    }

    void resetPosition() {
        m_encoder.setPosition(0);
    }

}

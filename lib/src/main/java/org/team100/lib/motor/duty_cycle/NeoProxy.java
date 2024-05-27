package org.team100.lib.motor.duty_cycle;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.Rev100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * A very simple wrapper around a CANSparkMax that only supports duty-cycle
 * output.
 * 
 * This makes the code that uses it easier to test.
 */
public class NeoProxy implements Motor100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;

    private final CANSparkMax m_motor;

    public NeoProxy(
            String name,
            int canId,
            boolean brakeMode,
            int currentLimit) {
        m_name = Names.append(name, this);
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        Rev100.currentConfig(m_motor, currentLimit);
        m_motor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    private void set(double speed) {
        m_motor.set(speed);
        t.log(Level.TRACE, m_name, "DUTY", m_motor.getAppliedOutput());
        t.log(Level.TRACE, m_name, "AMPS", m_motor.getOutputCurrent());
    }

    @Override
    public String getGlassName() {
        return "NeoProxy";
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
}

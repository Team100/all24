package org.team100.lib.motor.arm;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class JointMotor implements Motor100<Angle> {
    private final Telemetry t = Telemetry.get();
    private final CANSparkMax m_motor;
    private final String m_name;

    public JointMotor(String name, int canId, int currentLimit) {
        m_name = name + "/joint motor";
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(currentLimit);
        m_motor.setSecondaryCurrentLimit(currentLimit);
        m_motor.setIdleMode(IdleMode.kBrake);
        t.log(Level.DEBUG, m_name + "/Device ID", m_motor.getDeviceId());
    }

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
    }

    @Override
    public void setVelocity(double velocity, double accel) {
        throw new UnsupportedOperationException();
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

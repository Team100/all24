package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * Swerve steering motor using PWM control.
 */
public class PWMTurningMotor implements Motor100<Angle> {
    private final Telemetry t = Telemetry.get();
    private final PWMMotorController m_motor;
    private final String m_name;

    public PWMTurningMotor(String name, int channel) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = new VictorSP(channel);
        m_motor.setInverted(true);
        m_name = String.format("/%s/PWM Turning Motor", name);
        t.log(Level.DEBUG, m_name + "/Device ID", channel);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.DEBUG, m_name + "/Output", output);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * PWM does not support closed-loop velocity control. 
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2) {
        throw new UnsupportedOperationException("PWM turning motor does not implement closed loop velocity.");
    }

    @Override
    public void close() {
        m_motor.close();
    }
}

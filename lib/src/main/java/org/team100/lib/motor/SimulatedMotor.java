package org.team100.lib.motor;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

public class SimulatedMotor<T> implements Motor100<T> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;

    public SimulatedMotor(String name) {
        m_name = name + "/simulated_motor";
    }

    private double m_velocity = 0;

    @Override
    public double get() {
        // TODO: this is wrong
        System.out.println("WRONG");
        return m_velocity;
    }

    @Override
    public void setDutyCycle(double output) {
        t.log(Level.DEBUG, m_name + "/duty_cycle", output);
        System.out.println("Unimplemented! SimulatedMotor.setDutyCycle");
    }

    @Override
    public void stop() {
        m_velocity = 0;
    }

    @Override
    public void setVelocity(double velocity, double accel) {
        m_velocity = velocity;
        // ignore accel
        t.log(Level.DEBUG, m_name + "/velocity", m_velocity);
    }

    public double getVelocity() {
        t.log(Level.DEBUG, m_name + "/velocity", m_velocity);
        return m_velocity;
    }

}

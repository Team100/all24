package org.team100.lib.motor.turning;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class PWMTurningMotor implements TurningMotor {
    private final Telemetry t = Telemetry.get();
    private final PWMMotorController m_motor;
    private final String m_name;

    public PWMTurningMotor(String name, int channel) {
        m_motor = new VictorSP(channel);
        m_motor.setInverted(true);
        m_name = String.format("/PWM Turning Motor %s", name);
        t.log(m_name + "/Device ID",  channel);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.set(output);
        t.log(m_name + "/Output", output);
    }

    @Override
    // THIS DOES NOT ACTUALLY SET PID This is just here for the other turning motors
    // TODO fix this
    public void setPIDVelocity(double output, double Accel) {
        set(output);
    }

    @Override
    public void setPIDPosition(double output) {
        set(output);
    }
}

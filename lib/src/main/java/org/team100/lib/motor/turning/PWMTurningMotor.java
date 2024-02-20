package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * Swerve steering motor using PWM control.
 */
public class PWMTurningMotor implements Motor100<Angle100> {
    /** Very much not calibrated. */
    private static final double kV = 0.1;
    private final Telemetry t = Telemetry.get();
    private final PWMMotorController m_motor;
    private final String m_name;

    public PWMTurningMotor(String name, int channel) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = new VictorSP(channel);
        m_motor.setInverted(true);
        m_name = Names.append(name, this);
        t.log(Level.TRACE, m_name, "Device ID", channel);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "desired duty cycle [-1,1]", output);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Velocity is just kV feedforward and that's all.
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2) {
        m_motor.set(kV * outputRad_S);
    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void periodic() {
        //
    }
}

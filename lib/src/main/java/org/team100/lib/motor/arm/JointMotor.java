package org.team100.lib.motor.arm;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

/**
 * Arm motor from 2023.
 * 
 * Velocity is feedforward only.
 */
public class JointMotor implements Motor100<Angle100> {
    /** Very much not calibrated. */
    private static final double kV = 0.1;
    private static final double kTNm_amp = 0.02; // https://motors.vex.com/
    private final Telemetry t = Telemetry.get();
    private final CANSparkMax m_motor;
    private final String m_name;

    /**
     * @param name         may not start with a slash.
     * @param canId
     * @param currentLimit
     */
    public JointMotor(String name, int canId, int currentLimit) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setSmartCurrentLimit(currentLimit);
        m_motor.setSecondaryCurrentLimit(currentLimit);
        m_motor.setIdleMode(IdleMode.kBrake);

        // reduce total velocity measurement delay
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

        m_name = Names.append(name, this);

        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceId());
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
    }

    /**
     * Velocity is just kV feedforward and that's all.
     * 
     * @param velocity used with kV to get duty cycle
     * @param accel    ignored
     */
    @Override
    public void setVelocity(double velocity, double accel) {
        m_motor.set(kV * velocity);
    }

    /**
     * Velocity is just kV feedforward and that's all.
     * 
     * @param velocity used with kV to get duty cycle
     * @param accel    ignored
     * @param torque   ignored
     */
    @Override
    public void setVelocity(double velocity, double accel, double torque) {
        m_motor.set(kV * velocity);
    }

    @Override
    public double getTorque() {
        return kTNm_amp * m_motor.getOutputCurrent();
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

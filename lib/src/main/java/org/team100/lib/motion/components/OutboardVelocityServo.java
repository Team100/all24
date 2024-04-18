package org.team100.lib.motion.components;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.Timer;

/**
 * Passthrough to outboard closed-loop velocity control.
 */
public class OutboardVelocityServo<T extends Measure100> implements VelocityServo<T> {
    private final Telemetry t = Telemetry.get();
    private final Motor100<T> m_motor;
    private final Encoder100<T> m_encoder;
    private final String m_name;

    // for calculating acceleration
    private double previousSetpoint = 0;
    private double prevTime;
    private double m_setpoint;

    /**
     * @param name    may not start with slash
     * @param motor
     * @param encoder
     */
    public OutboardVelocityServo(String name, Motor100<T> motor, Encoder100<T> encoder) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = motor;
        m_encoder = encoder;
        m_name = Names.append(name, this);
    }

    @Override
    public void reset() {
        prevTime = Timer.getFPGATimestamp();
        // ALERT!  @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want.  but this might be wrong
        // for some other reason
        // m_encoder.reset();
    }

    @Override
    public void setVelocity(double setpoint) {
        if (Double.isNaN(setpoint))
            throw new IllegalArgumentException("setpoint is NaN");
        m_setpoint = setpoint;
        m_motor.setVelocity(setpoint, accel(setpoint));
        t.log(Level.TRACE, m_name, "Desired setpoint", setpoint);
    }

    /** Direct control for testing. */
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_motor.setDutyCycle(dutyCycle);
        t.log(Level.TRACE, m_name, "Desired duty cycle [-1,1]", dutyCycle);
    }

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    @Override
    public double getVelocity() {
        return m_encoder.getRate();
    }

    @Override
    public double getTorque() {
        return m_motor.getTorque();
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public double getDistance() {
        return m_encoder.getPosition();
    }

    /** For testing */
    @Override
    public double getSetpoint() {
        return m_setpoint;
    }

    ////////////////////////////////////////////////

    /**
     * there will be some jitter in dt, which will result in a small amount of
     * jitter in acceleration, and since this is a trailing difference there will be
     * a tiny bit of delay, compared to the actual profile. If this is
     * a problem, rewrite the profile class to expose the acceleration state and use
     * that instead.
     */
    private double accel(double setpoint) {
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        prevTime = now;
        double accel = (setpoint - previousSetpoint) / dt;
        previousSetpoint = setpoint;
        return accel;
    }
}

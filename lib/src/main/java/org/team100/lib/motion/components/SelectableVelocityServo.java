package org.team100.lib.motion.components;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;

/**
 * This velocity servo uses two methods, selectable via experiment at runtime:
 * 
 * 1. Passthrough to outboard closed-loop velocity control, if supported.
 * 2. Onboard PIDF, for dumb controllers.
 * 
 * If you don't need to switch between these at runtime, then choose a different
 * implementation.
 */
public class SelectableVelocityServo<T extends Measure100> implements VelocityServo<T> {
    private static final double kDeadband = 0.03;

    private final Telemetry t = Telemetry.get();

    private final Motor100<T> m_motor;
    private final Encoder100<T> m_encoder;
    private final PIDController m_controller;
    private final SimpleMotorFeedforward m_feedforward;
    private final String m_name;

    // for calculating acceleration
    private double previousSetpoint = 0;
    private double prevTime;
    private double m_setpoint;

    /**
     * @param name        may not start with slash
     * @param motor
     * @param encoder
     * @param controller
     * @param feedforward
     */
    public SelectableVelocityServo(
            String name,
            Motor100<T> motor,
            Encoder100<T> encoder,
            PIDController controller,
            SimpleMotorFeedforward feedforward) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = motor;
        m_encoder = encoder;
        m_controller = controller;
        m_feedforward = feedforward;
        m_name = Names.append(name, this);
    }

    @Override
    public void reset() {
        prevTime = Timer.getFPGATimestamp();
        m_encoder.reset();
    }



    @Override
    public void setVelocity(double setpoint) {
        if (Double.isNaN(setpoint))
            throw new IllegalArgumentException("setpoint is NaN");
        m_setpoint = setpoint;
        if (Experiments.instance.enabled(Experiment.UseClosedLoopVelocity)) {
            offboard(setpoint);
        } else {
            onboard(setpoint);
        }
        t.log(Level.DEBUG, m_name, "Desired setpoint", setpoint);
        t.log(Level.DEBUG, m_name, "Controller Setpoint", m_controller.getSetpoint());
        t.log(Level.DEBUG, m_name, "Controller Speed Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name, "Controller Accel Error", m_controller.getVelocityError());
    }

    /** Direct control for testing. */
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_motor.setDutyCycle(dutyCycle);
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

    private void offboard(double velocity) {
        if (Double.isNaN(velocity))
            throw new IllegalArgumentException("setpoint is NaN");
        m_motor.setVelocity(velocity, accel(velocity));
    }

    private void onboard(double velocity) {
        double u_FB = m_controller.calculate(getVelocity(), velocity);
        double u_FF = m_feedforward.calculate(velocity, accel(velocity));
        double u_TOTAL = u_FB + u_FF;
        u_TOTAL = MathUtil.applyDeadband(u_TOTAL, kDeadband, 1);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -1, 1);
        m_motor.setDutyCycle(u_TOTAL);
        t.log(Level.DEBUG, m_name, "u_FB", u_FB);
        t.log(Level.DEBUG, m_name, "u_FF", u_FF);
        t.log(Level.DEBUG, m_name, "u_TOTAL", u_TOTAL);
    }

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

    @Override
    public void periodic() {
        m_motor.periodic();
        m_encoder.periodic();
    }
}

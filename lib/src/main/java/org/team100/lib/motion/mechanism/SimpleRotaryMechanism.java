package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;

import edu.wpi.first.math.MathUtil;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The included encoder is the incremental motor encoder.
 */
public class SimpleRotaryMechanism implements RotaryMechanism, Glassy {
    private final BareMotor m_motor;
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;
    // LOGGER
    private DoubleLogger m_log_velocity;
    private DoubleLogger m_log_position;

    public SimpleRotaryMechanism(
            LoggerFactory parent,
            BareMotor motor,
            IncrementalBareEncoder encoder,
            double gearRatio) {
        LoggerFactory child = parent.child(this);
        m_motor = motor;
        m_encoder = encoder;
        m_gearRatio = gearRatio;
        m_log_velocity = child.doubleLogger(Level.TRACE, "velocity (rad_s)");
        m_log_position = child.doubleLogger(Level.TRACE, "position (rad)");
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.setDutyCycle(output);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_motor.setTorqueLimit(torqueNm / m_gearRatio);
    }

    @Override
    public void setVelocity(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    @Override
    public void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputTorqueNm) {
        m_motor.setPosition(
                outputPositionRad * m_gearRatio,
                outputVelocityRad_S * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    /** nearly cached */
    @Override
    public OptionalDouble getVelocityRad_S() {
        OptionalDouble velocityRad_S = m_encoder.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(velocityRad_S.getAsDouble() / m_gearRatio);
    }

    /** For checking calibration, very slow, do not use outside tests. */
    @Override
    public double getPositionBlockingRad() {
        return m_encoder.getPositionBlockingRad() / m_gearRatio;
    }

    /** nearly cached */
    @Override
    public OptionalDouble getPositionRad() {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(positionRad.getAsDouble() / m_gearRatio);
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void resetEncoderPosition() {
        m_encoder.reset();
    }

    /** This can be very slow, only use it on startup. */
    @Override
    public void setEncoderPosition(double positionRad) {
        double motorPositionRad = positionRad * m_gearRatio;
        m_encoder.setEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_velocity.log(() -> getVelocityRad_S().getAsDouble());
        m_log_position.log(() -> MathUtil.angleModulus(getPositionRad().getAsDouble()));
        m_motor.periodic();
        m_encoder.periodic();
    }

}

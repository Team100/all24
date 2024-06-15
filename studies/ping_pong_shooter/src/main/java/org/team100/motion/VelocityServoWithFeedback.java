package org.team100.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a continuous-rotation velocity-controlled servo with positional
 * feedback, such as the Parallax360.
 * 
 * Positional control is just proportional feedback.
 * 
 * Servo hardware includes an outboard feedback controller for velocity, which
 * probably uses proportional control and a deadband.
 * 
 * PWM parameters match the Parallax360.
 * 
 * The measurement signal is a 910hz PWM signal ranging from 2.7% to 97.1%.
 * 
 * TODO: use the positional input to feed one ball at a time, and add a selector
 * button to choose "semi-auto" or "full auto" mode.
 * 
 * @see https://www.pololu.com/product/3432
 * @see https://www.pololu.com/file/0J1395/900-00360-Feedback-360-HS-Servo-v1.2.pdf
 * @see https://www.princeton.edu/~mae412/TEXT/NTRAK2002/292-302.pdf
 * @see https://rocelec.widen.net/view/pdf/npfew4u7vv/M51660.pdf
 */
public class VelocityServoWithFeedback {
    // TODO: calibrate this
    private static final double kDistancePerRotation = 1.0;
    // full state gain: x, v
    // private static final double[] kK = new double[] { 1.0, 1.0 };
    private static final double[] kK = new double[] { 0.0, 0.0 };
    // maximum allowed output
    private static final double kMaxU = 1.0;

    private final String m_name;
    private final PWM m_pwm;
    private final DutyCycleEncoder m_encoder;

    // for velocity calculation
    private double m_position;
    private double m_velocity;
    private double m_time;

    public VelocityServoWithFeedback(
            String name,
            int pwmChannel,
            int encoderChannel) {
        m_name = name;
        m_pwm = new PWM(pwmChannel);
        // These parameters match the Parallax 360.
        m_pwm.setBoundsMicroseconds(1720, 1520, 1500, 1480, 1280);
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
        m_pwm.setSpeed(0.0);
        m_pwm.setZeroLatch();

        m_encoder = new DutyCycleEncoder(encoderChannel);
        m_encoder.setDutyCycleRange(0.027, 0.971);
        m_encoder.setDistancePerRotation(kDistancePerRotation);
    }

    /** Goal is not wrapped, might be far from measurement. */
    public void setPosition(double positionGoal) {
        SmartDashboard.putNumber(m_name + "/goal", positionGoal);
        // velocity goal is always zero
        final double velocityGoal = 0;

        double positionError = positionGoal - m_position;
        SmartDashboard.putNumber(m_name + "/positionError", positionError);

        double velocityError = velocityGoal - m_velocity;
        SmartDashboard.putNumber(m_name + "/velocityError", velocityError);

        // dot product of gains * errors
        double u_FB = -1.0 * (kK[0] * positionError + kK[1] * velocityError);
        u_FB = MathUtil.clamp(u_FB, -kMaxU, kMaxU);
        SmartDashboard.putNumber(m_name + "/u_FB", u_FB);

        m_pwm.setSpeed(u_FB);
    }

    /** @goal range [-1,1] */
    public void setSpeed(double goal) {
        m_pwm.setSpeed(goal);
    }

    public void disable() {
        m_pwm.setDisabled();
    }

    /** Calculates velocity with one-step finite difference: noisy. */
    public void periodic() {
        double position = m_encoder.getDistance();
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        double dx = position - m_position;
        m_position = position;
        m_velocity = dx / dt;
        m_time = now;
        SmartDashboard.putNumber(m_name + "/position", m_position);
        SmartDashboard.putNumber(m_name + "/velocity", m_velocity);
    }

}

package org.team100.motion;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Flywheel using PWM-controlled DC motor with a reduction gear and quadrature
 * encoders.
 * 
 * @see https://www.pololu.com/product/4789
 * @see https://www.pololu.com/product/2137
 * @see https://www.pololu.com/product/4761
 */
public class Flywheel {
    // full state gain: v, a
    private static final double[] kK = new double[] { 1.0, 1.0 };

    // TODO: calibrate this
    private static final double kV = 2.7;
    /**
     * gear ratio for pololu 4789 is 15.24884259
     * 
     * for magnet 2599, a quadrature encoder yields 3 four-phase cycles per
     * revolution.
     * 
     * so ticks per turn is 45.74652778
     */
    private static final double TICKS_PER_TURN = 45.74652778;

    private final PWM m_pwm;
    private final Encoder m_encoder;

    // for accel calculation
    private double m_velocity;
    private double m_accel;
    private double m_time;

    public Flywheel(
            int pwmChannel,
            int encoderChannelA,
            int encoderChannelB) {
        m_pwm = new PWM(pwmChannel);
        m_encoder = new Encoder(encoderChannelA, encoderChannelB, false, EncodingType.k1X);
        m_encoder.setDistancePerPulse(60 / TICKS_PER_TURN);
    }

    public void setVelocity(double velocityGoal) {
        SmartDashboard.putNumber("goal", velocityGoal);
        // accel goal is always zero.
        final double accelGoal = 0;

        double velocityError = velocityGoal - m_velocity;
        SmartDashboard.putNumber("velocityError", velocityError);

        double accelError = accelGoal - m_accel;
        SmartDashboard.putNumber("accelError", accelError);

        double u_FB = kK[0] * velocityError + kK[1] * accelError;

        double pulseUs = kV * velocityGoal + u_FB;
        int u_TOTAL = (int) MathUtil.clamp(pulseUs, 0, 4095);
        m_pwm.setPulseTimeMicroseconds(u_TOTAL);
    }

    public void periodic() {
        double velocity = m_encoder.getRate();
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        double dv = velocity - m_velocity;
        m_velocity = velocity;
        m_accel = dv / dt;
        m_time = now;
        SmartDashboard.putNumber("velocity", m_velocity);
        SmartDashboard.putNumber("accel", m_accel);
    }

}

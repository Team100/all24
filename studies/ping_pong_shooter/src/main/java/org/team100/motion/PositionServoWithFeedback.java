package org.team100.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a positional servo with positional feedback, such as the Feetch
 * FS51606B-FB.
 * 
 * The positional measurement is only used here for initialization, to avoid
 * jumping at startup.
 * 
 * @see https://www.pololu.com/product/3442
 * @see https://www.princeton.edu/~mae412/TEXT/NTRAK2002/292-302.pdf
 * @see https://rocelec.widen.net/view/pdf/npfew4u7vv/M51660.pdf
 */
public class PositionServoWithFeedback {
    // The scale and offset are derived here:
    // https://docs.google.com/spreadsheets/d/1I54xvnMfn0FJnX9UgfJ5rrI9iUO81Urhvoy3D3JJbsQ
    private static final double kEncoderScale = 3.148148;
    private static final double kEncoderOffset = -0.407222;

    private final String m_name;
    private final PWM m_pwm;
    private final AnalogInput m_input;
    private final AnalogEncoder m_encoder;
    private final SlewRateLimiter m_profile;

    public PositionServoWithFeedback(
            String name,
            int pwmChannel,
            int encoderChannel,
            double slewRatePerSec) {
        m_name = name;
        m_pwm = new PWM(pwmChannel);
        // These parameters match the Feetch FS5106B-FB.
        m_pwm.setBoundsMicroseconds(2300, 0, 0, 0, 700);
        m_pwm.setPeriodMultiplier(PeriodMultiplier.k4X);
        m_input = new AnalogInput(encoderChannel);
        m_encoder = new AnalogEncoder(m_input);
        m_profile = new SlewRateLimiter(slewRatePerSec);
    }

    /** @param goal position fraction of servo motion [0,1] */
    public void setPosition(double goal) {
        goal = MathUtil.clamp(goal, 0, 1);
        SmartDashboard.putNumber(m_name + "/goal [0,1]", goal);

        double setpoint = m_profile.calculate(goal);
        SmartDashboard.putNumber(m_name + "/setpoint [0,1]", setpoint);
        m_pwm.setPosition(setpoint);
    }

    /** returns the current position in the range [0,1] */
    public double getPosition() {
        double measurement = MathUtil.clamp(m_encoder.getAbsolutePosition() * kEncoderScale + kEncoderOffset, 0, 1);
        SmartDashboard.putNumber(m_name + "/measurement [0,1]", measurement);
        return measurement;
    }

    /** Initialize the pwm and profile to the current measurement. */
    public void enable() {
        double measurement = getPosition();
        m_profile.reset(measurement);
        setPosition(measurement);
    }

    public void disable() {
        m_pwm.setDisabled();
    }

    public void periodic() {
        getPosition(); // just to log the measurement
    }

    public void close() {
        m_input.close();
    }
}

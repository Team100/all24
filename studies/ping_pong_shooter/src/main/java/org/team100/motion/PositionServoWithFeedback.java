package org.team100.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a positional servo with positional feedback.
 * 
 * The measurement signal is an analog voltage that varies from some minimum to
 * some maximum, representing the whole range of motion. This needs to be
 * calibrated.
 * 
 * For simplicity's sake:
 * 
 * the control input is a fraction of the total range of motion, i.e. [0,1]
 *
 * the "profile" is simply a slew rate limiter, which results in infinite
 * acceleration.  TODO: Maybe get rid of this entirely?
 * 
 * Servo hardware includes an outboard feedback controller, so there is no
 * feedback here.
 * 
 * Velocity is not measured or controlled by the outboard controller.
 * 
 * A typical servo control method is simply proportional feedback with a
 * deadband.
 * 
 * Note this class does not implement "motor safety" i.e. it's possible to move
 * the servo while disabled.
 * 
 * The PWM parameters match the Feetch FS5160B.
 * 
 * @see https://www.pololu.com/product/3442
 * @see https://www.princeton.edu/~mae412/TEXT/NTRAK2002/292-302.pdf
 * @see https://rocelec.widen.net/view/pdf/npfew4u7vv/M51660.pdf
 */
public class PositionServoWithFeedback {
    private static final double kEncoderScale = 1.0;
    private static final double kEncoderOffset = 0.0;
    private final PWM m_pwm;
    private final AnalogInput m_input;
    private final AnalogEncoder m_encoder;
    private final SlewRateLimiter m_profile;

    public PositionServoWithFeedback(
            int pwmChannel,
            int encoderChannel,
            double slewRateRad_S) {
        m_pwm = new PWM(pwmChannel);
        m_pwm.setBoundsMicroseconds(2300, 0, 0, 0, 700);
        m_pwm.setPeriodMultiplier(PeriodMultiplier.k4X);
        m_input = new AnalogInput(encoderChannel);
        m_encoder = new AnalogEncoder(m_input);
        m_profile = new SlewRateLimiter(slewRateRad_S);
    }

    /** Initialize the pwm and profile to the current measurement. */
    public void init() {
        double measurement = get();
        m_profile.reset(measurement);
        set(measurement);
    }

    /** @param goal position fraction of servo motion [0,1] */
    public void set(double goal) {
        goal = MathUtil.clamp(goal, 0, 1);
        SmartDashboard.putNumber("goal [0,1]", goal);

        double setpoint = m_profile.calculate(goal);
        SmartDashboard.putNumber("setpoint [0,1]", setpoint);
        m_pwm.setPosition(setpoint);
    }

    public double get() {
        double measurement = MathUtil.clamp(m_encoder.getAbsolutePosition() * kEncoderScale + kEncoderOffset, 0, 1);
        SmartDashboard.putNumber("measurement [0,1]", measurement);
        return measurement;
    }

    public void close() {
        m_input.close();
    }

}

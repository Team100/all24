package org.team100.motion;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Flywheel using PWM-controlled DC motor with a reduction gear and quadrature
 * encoders.
 * 
 * The controller is a simple inverter, so it is capable of bidirectional output
 * but only some inputs are connected. IN1 and IN2 drive phase A, IN3 and IN4
 * drive B.
 * 
 * IN1: pulled down, wired to gnd
 * IN2: puled up, wired to pwm
 * IN3: pulled down, wired to gnd
 * IN4: pulled up, wired to pwm
 * 
 * The output is "clockwise" when the two controls are both high,
 * "counterclockwise" when both low, and "braking" when the two inputs are
 * different.
 * 
 * The RoboRIO DIO ports include "pull ups" so before being activated they'll
 * float high.
 * 
 * So they should be wired to the "pulled up" inputs of the controller, and the
 * other inputs should be wired to ground.
 * 
 * so the trouble is that if you say "duty cycle 0" then it seems to sit low,
 * which is the opposite of what you want. i guess i could just fix it here, 1 -
 * dc.
 * 
 * There's also an "inhibit" input, pulled low, active low.
 * 
 * I think the RoboRIO output "on" time is 0-4096 microseconds, and the minimum
 * period is 5050 microseconds, so the maximum duty cycle is 0.81%
 * 
 * @see https://www.pololu.com/product/4789
 * @see https://www.pololu.com/product/2137
 * @see https://www.pololu.com/product/4761
 */
public class Flywheel {
    public static enum Direction {
        Forward, Reverse
    }

    private static final int kPWMFreqHz = 10000;

    // full state gain: v, a
    // accel is very noisy, so don't use it until we clean it up somehow.
    private static final double[] kK = new double[] { 0.05, 0.0 };
    // private static final double[] kK = new double[] { 1.0, 1.0 };

    // TODO: calibrate this
    // private static final double kV = 0.0;
    // free speed at the encoder appears to be about 2500 rpm or 40 rev/s
    private static final double kV = 0.024;
    /**
     * gear ratio for pololu 4789 is 15.24884259
     * 
     * for magnet 2599, a quadrature encoder yields 3 four-phase cycles per
     * revolution.
     * 
     * so ticks per turn is 45.74652778
     */
    private static final double TICKS_PER_TURN = 45.74652778;

    private final String m_name;
    private final DigitalOutput m_output;
    private final Direction m_direction;
    private final Encoder m_encoder;

    // for accel calculation
    private double m_velocityRev_S;
    private double m_accelRev_S2;
    private double m_time;

    public Flywheel(
            String name,
            int outputChannel,
            Direction direction,
            int encoderChannelA,
            int encoderChannelB) {
        m_name = name;
        m_output = new DigitalOutput(outputChannel);
        m_output.setPWMRate(kPWMFreqHz);
        m_direction = direction;
        m_encoder = new Encoder(encoderChannelA, encoderChannelB, false, EncodingType.k1X);
        // distance per pulse in turns is 1/ticksperturn.
        m_encoder.setDistancePerPulse(1 / TICKS_PER_TURN);
    }

    /** @param velocityGoalRev_S in revs per second */
    public void setVelocity(double velocityGoalRev_S) {
        SmartDashboard.putNumber(m_name + "/velocityGoal (rev_s)", velocityGoalRev_S);
        // accel goal is always zero.
        final double accelGoalRev_S2 = 0;

        double velocityErrorRev_S = velocityGoalRev_S - m_velocityRev_S;
        SmartDashboard.putNumber(m_name + "/velocityError (rev_s)", velocityErrorRev_S);

        double accelErrorRev_S2 = accelGoalRev_S2 - m_accelRev_S2;
        SmartDashboard.putNumber(m_name + "/accelError (rev_s2)", accelErrorRev_S2);

        double FF = kV * velocityGoalRev_S;
        SmartDashboard.putNumber(m_name + "/FF", FF);

        double u_FB = kK[0] * velocityErrorRev_S + kK[1] * accelErrorRev_S2;
        SmartDashboard.putNumber(m_name + "/u_FB", u_FB);

        double u_TOTAL = MathUtil.clamp(FF + u_FB, 0, 1);
        SmartDashboard.putNumber(m_name + "/u_TOTAL", u_TOTAL);
        // "high" is off, so we need to invert the output.
        m_output.updateDutyCycle(1.0 - u_TOTAL);
    }

    public void enable() {
        // "inactive" is high
        m_output.enablePWM(1);
    }

    public void disable() {
        m_output.disablePWM();
        // "inactive" is high
        m_output.set(true);
    }

    public void periodic() {
        // uses distance per pulse to produce distance per second.
        double velocityRev_S = m_encoder.getRate() * (m_direction == Direction.Forward ? 1.0 : -1.0);
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        double dv = velocityRev_S - m_velocityRev_S;
        m_velocityRev_S = velocityRev_S;
        m_accelRev_S2 = dv / dt;
        m_time = now;
        SmartDashboard.putNumber(m_name + "/velocity (rev_s)", m_velocityRev_S);
        SmartDashboard.putNumber(m_name + "/accel (rev_s2)", m_accelRev_S2);
    }

}

package org.team100.frc2024.commands.climber;

import java.util.OptionalDouble;

import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LimitedLinearMechanism;
import org.team100.lib.util.Timer100;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * The zeroing process has four states, which are indicated by the states of the
 * two timers:
 * 
 * <pre>
 * initial: set the effort to low and start the start timer
 * 
 * starting state (start timer running, end timer not running): run at the target speed
 * 
 * transition: when the start timer runs out:
 * 
 * running state (start timer > limit, end timer not running): run at the target speed
 * 
 * transition: when the actual speed is below the threshold, start the stop timer
 * 
 * stopping state (start timer > limit, end timer running): run at the target speed
 * 
 * transition: when the stop timer runs out, zero the mechanism
 * 
 * done state (start timer > limit, end timer > limit): set speed to zero
 * </pre>
 */
public class HomeClimber extends Command implements Glassy {
    /** Target speed is negative, i.e. down. */
    private static final double kTargetSpeedM_S = -0.03;
    /** Threshold is unsigned. */
    private static final double kThresholdSpeedM_S = 0.003;
    /** Wait 0.5s to start moving. */
    private static final double kStartTimeS = 0.2;
    /** Wait 0.5s to stop moving. */
    private static final double kHomeTimeS = 0.2;

    private final ClimberSubsystem m_climber;
    private final Timer100 m_leftStart;
    private final Timer100 m_leftDone;
    private final Timer100 m_rightStart;
    private final Timer100 m_rightDone;

    // LOGGERS
    private final DoubleLogger m_log_left_start_timer_s;
    private final DoubleLogger m_log_right_start_timer_s;
    private final DoubleLogger m_log_left_done_timer_s;
    private final DoubleLogger m_log_right_done_timer_s;

    public HomeClimber(LoggerFactory logger, ClimberSubsystem climber) {
        LoggerFactory child = logger.child(this);
        m_log_left_start_timer_s = child.doubleLogger(Level.TRACE, "left start timer (s)");
        m_log_right_start_timer_s = child.doubleLogger(Level.TRACE, "right start timer (s)");
        m_log_left_done_timer_s = child.doubleLogger(Level.TRACE, "left done timer (s)");
        m_log_right_done_timer_s = child.doubleLogger(Level.TRACE, "right done timer (s)");

        m_climber = climber;
        m_leftStart = new Timer100();
        m_leftDone = new Timer100();
        m_rightStart = new Timer100();
        m_rightDone = new Timer100();
    }

    @Override
    public void initialize() {
        init(m_leftStart, m_leftDone);
        init(m_rightStart, m_rightDone);
        m_climber.setHomingForce();
    }

    @Override
    public void execute() {
        oneSide(m_log_left_start_timer_s, m_log_left_done_timer_s, m_leftStart, m_leftDone, m_climber.getLeft());
        oneSide(m_log_right_start_timer_s, m_log_right_done_timer_s, m_rightStart, m_rightDone, m_climber.getRight());
    }

    private static void init(Timer100 start, Timer100 done) {
        done.stop();
        done.reset();
        start.restart();
    }

    private void oneSide(
            DoubleLogger start_timer_s,
            DoubleLogger done_timer_s,
            Timer100 start,
            Timer100 done,
            LimitedLinearMechanism mech) {
        start_timer_s.log(start::get);
        done_timer_s.log(done::get);

        OptionalDouble opt = mech.getVelocityM_S();
        if (opt.isEmpty()) {
            Util.warn("HomeClimber: can't home, broken velocity sensor!");
            mech.stop();
            return;
        }
        if (start.get() < kStartTimeS) {
            // starting up, run the mech unconditionally
            mech.setVelocityUnlimited(kTargetSpeedM_S, 0, 0);
        } else {
            if (done.isStopped()) {
                // have not noticed speed decline, keep going.
                mech.setVelocityUnlimited(kTargetSpeedM_S, 0, 0);
                if (Math.abs(opt.getAsDouble()) < kThresholdSpeedM_S) {
                    // mech seems stopped, start the done timer
                    done.start();
                }
            } else {
                if (done.get() < kHomeTimeS) {
                    // speed is below the threshold, wait a bit.
                    mech.setVelocityUnlimited(kTargetSpeedM_S, 0, 0);
                } else {
                    // speed has been below the threshold for awhile, so we're done
                    mech.stop();
                    mech.resetEncoderPosition();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_leftDone.get() > kHomeTimeS && m_rightDone.get() > kHomeTimeS;
    }


}

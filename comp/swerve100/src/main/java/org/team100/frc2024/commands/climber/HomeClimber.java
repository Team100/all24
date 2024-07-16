package org.team100.frc2024.commands.climber;

import java.util.OptionalDouble;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.lib.telemetry.SupplierLogger;
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
public class HomeClimber extends Command {
    private static final double kTargetSpeedM_S = 0.02;
    private static final double kThresholdSpeedM_S = 0.002;
    private static final double kStartTimeS = 0.5;
    private static final double kHomeTimeS = 0.5;

    private final SupplierLogger m_logger;
    private final ClimberSubsystem m_climber;
    private final Timer100 m_leftStartTimer;
    private final Timer100 m_leftDoneTimer;
    private final Timer100 m_rightStartTimer;
    private final Timer100 m_rightDoneTimer;

    public HomeClimber(
            SupplierLogger logger,
            ClimberSubsystem climber) {
        m_logger = logger;
        m_climber = climber;
        m_leftStartTimer = new Timer100();
        m_leftDoneTimer = new Timer100();
        m_rightStartTimer = new Timer100();
        m_rightDoneTimer = new Timer100();
    }

    @Override
    public void initialize() {
        init(m_leftStartTimer, m_leftDoneTimer);
        init(m_rightStartTimer, m_rightDoneTimer);
        m_climber.setHomingForce();
    }

    @Override
    public void execute() {
        oneSide(
                m_leftStartTimer,
                m_leftDoneTimer,
                m_climber::setLeftVelocityM_S,
                m_climber::getLeftVelocity,
                m_climber::zeroLeft);
        oneSide(
                m_rightStartTimer,
                m_rightDoneTimer,
                m_climber::setRightVelocityM_S,
                m_climber::getRightVelocity,
                m_climber::zeroRight);
    }

    private static void init(Timer100 start, Timer100 done) {
        done.stop();
        done.reset();
        start.restart();
    }

    private static void oneSide(
            Timer100 start,
            Timer100 done,
            DoubleConsumer setter,
            Supplier<OptionalDouble> getter,
            Runnable zero) {
        OptionalDouble opt = getter.get();
        if (opt.isEmpty()) {
            Util.warn("HomeClimber: can't home, broken velocity sensor!");
            return;
        }
        if (start.get() < kStartTimeS) {
            // starting up, run the mech unconditionally
            setter.accept(kTargetSpeedM_S);
        } else {
            if (done.isStopped()) {
                // have not noticed speed decline, keep going.
                setter.accept(kTargetSpeedM_S);
                if (Math.abs(opt.getAsDouble()) < kThresholdSpeedM_S) {
                    // mech seems stopped, start the done timer
                    done.start();
                }
            } else {
                if (done.get() < kHomeTimeS) {
                    // speed is below the threshold, wait a bit.
                    setter.accept(kTargetSpeedM_S);
                } else {
                    // speed has been below the threshold for awhile, so we're done
                    setter.accept(0);
                    zero.run();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_leftDoneTimer.get() > kHomeTimeS && m_rightDoneTimer.get() > kHomeTimeS;
    }

    @Override
    public void end(boolean interrupted) {
    }

}

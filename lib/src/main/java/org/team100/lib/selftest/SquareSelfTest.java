package org.team100.lib.selftest;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Verifies a one-meter square trajectory.
 * 
 * The command under test should produce a 1 m square, 2 s on each side.
 */
@ExcludeFromJacocoGeneratedReport
public class SquareSelfTest extends Command {
    private static final double kProfileSec = 2;
    private static final double kMaxDistance = 1.75;
    private static final double kSteerSec = 0.34;
    /** 2 s per side */
    private static final double kExpectedDuration = 4 * kProfileSec + 3 * kSteerSec;
    private final SwerveDriveSubsystem m_drivetrain;
    private final SelfTestListener m_listener;
    private final Timer m_timer;
    private boolean terminate = false;
    private Pose2d m_initial;

    public SquareSelfTest(SwerveDriveSubsystem drivetrain, SelfTestListener listener) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_initial = m_drivetrain.getPose();
        m_timer.start();
    }

    @Override
    public void execute() {
        Pose2d state = m_drivetrain.getPose();
        double x = state.getX() - m_initial.getX();
        double y = state.getY() - m_initial.getY();
        double t = m_timer.get();

         if (GeometryUtil.distance(state, m_initial) > kMaxDistance) {
            m_listener.fail(this, "Too far from initial pose");
            terminate = true;
            return;
        }

        if (t < kProfileSec) {
            // first side takes 2 s
            if (MathUtil.isNear(0, y, 0.1) && between(x, 0, 1, 0.1))
                return;
            m_listener.fail(this, "side 1 fail: t %5.3f x %5.3f y %5.3f", t, x, y);
        } else if (t < 2 * kProfileSec + kSteerSec) {
            // second side starts after turning
            if (MathUtil.isNear(1, x, 0.1) && between(y, 0, 1, 0.1))
                return;
            m_listener.fail(this, "side 2 fail: t %5.3f x %5.3f y %5.3f", t, x, y);
        } else if (t < 3 * kProfileSec + 2 * kSteerSec) {
            // third side, another steering delay
            if (MathUtil.isNear(1, y, 0.1) && between(x, 0, 1, 0.1))
                return;
            m_listener.fail(this, "side 3 fail: t %5.3f x %5.3f y %5.3f", t, x, y);
        } else if (t < kExpectedDuration) {
            // fourth side, another steering delay
            if (MathUtil.isNear(0, x, 0.1) && between(y, 0, 1, 0.1))
                return;
            m_listener.fail(this, "side 4 fail: t %5.3f x %5.3f y %5.3f", t, x, y);
        }

        // if the command under test leaves the expected trajectory, or the required
        // time has elapsed, it should be terminated.

        terminate = true;
    }

    private boolean between(double x, double l, double h, double tolerance) {
        return x > (l - tolerance) && x < (h + tolerance);
    }

    @Override
    public boolean isFinished() {
        return terminate || m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        double time = m_timer.get();
        // the time is determined by this class since the command under test never
        // terminates.
        String fmt = "final time expected %5.3f actual %5.3f";
        if (MathUtil.isNear(kExpectedDuration, time, 0.1))
            m_listener.pass(this, fmt, kExpectedDuration, time);
        else
            m_listener.fail(this, fmt, kExpectedDuration, time);

        // should end up near where we started.
        Pose2d state = m_drivetrain.getPose();
        double x = state.getX() - m_initial.getX();
        double y = state.getY() - m_initial.getY();
        double theta = state.getRotation().getRadians() - m_initial.getRotation().getRadians();

        fmt = "final x expected 0.0 actual: %5.3f";
        if (MathUtil.isNear(0, x, 0.1))
            m_listener.pass(this, fmt, x);
        else
            m_listener.fail(this, fmt, x);

        fmt = "final y expected 0.0 actual: %5.3f";
        if (MathUtil.isNear(0, y, 0.1))
            m_listener.pass(this, fmt, y);
        else
            m_listener.fail(this, fmt, y);

        fmt = "final theta expected 0.0 actual: %5.3f";
        if (MathUtil.isNear(0, theta, 0.1))
            m_listener.pass(this, fmt, theta);
        else
            m_listener.fail(this, fmt, theta);
    }
}

package org.team100.lib.selftest;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This test drives +x for 1 m at about 0.5 m/s
 */
@ExcludeFromJacocoGeneratedReport
public class DriveManuallySelfTest extends Command {
    private static final double kExpectedDuration = 2;
    private static final double kMaxDistance = 1.5;

    private final SwerveDriveSubsystem m_drivetrain;
    private final SelfTestListener m_listener;
    private final Timer m_timer;
    
    private boolean terminate = false;
    private Pose2d m_initial;

    public DriveManuallySelfTest(SwerveDriveSubsystem drivetrain, SelfTestListener listener) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
    }

    public Twist2d treatment() {
        // allow 0.1s to move the wheels 90 degrees; at 0.1m/s for 0.1s this should only
        // move 0.01m.
        if (m_timer.get() < 0.1)
            return new Twist2d(0.1, 0, 0);
        // then move in +x at 0.5 m/s for 2 s so the result should be about 1m
        return new Twist2d(0.5, 0, 0);
    }

    @Override
    public void initialize() {
        m_initial = m_drivetrain.getPose();
        m_timer.restart();
        terminate = false;
    }

    @Override
    public void execute() {
        if (GeometryUtil.distance(m_drivetrain.getPose(), m_initial) > kMaxDistance) {
            m_listener.fail(this, "Too far from initial pose");
            terminate = true;
        }
    }

    @Override
    public boolean isFinished() {
        return terminate || m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        double time = m_timer.get();
        String fmt = "final time expected %5.3f actual %5.3f";
        double expected = kExpectedDuration;
        if (MathUtil.isNear(expected, time, 0.1))
            m_listener.pass(this, fmt, expected, time);
        else
            m_listener.fail(this, fmt, expected, time);

        Pose2d state = m_drivetrain.getPose();

        // 0.5 m/s for 2 s is roughly 1m
        expected = 1;
        double actual = state.getX() - m_initial.getX();
        fmt = "final x expected %5.3f actual %5.3f";
        // within 10 cm.
        if (MathUtil.isNear(expected, actual, 0.1))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);

        // no treatment for y
        expected = 0;
        actual = state.getY() - m_initial.getY();
        fmt = "final y expected %5.3f actual %5.3f";
        if (MathUtil.isNear(expected, actual, 0.1))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);

        // no treatment in theta
        expected = 0;
        actual = state.getRotation().getRadians() - m_initial.getRotation().getRadians();
        fmt = "final rotation expected %5.3f actual %5.3f";
        if (MathUtil.isNear(expected, actual, 0.1))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);
    }

}

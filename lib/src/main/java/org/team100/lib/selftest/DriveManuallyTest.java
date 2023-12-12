package org.team100.lib.selftest;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

@ExcludeFromJacocoGeneratedReport
public class DriveManuallyTest extends Command {
    // two second duration
    private static final double kExpectedDuration = 2;
    private final SwerveDriveSubsystem m_drivetrain;
    private final TestListener m_listener;
    private final Timer m_timer;
    private boolean terminate = false;
    private Pose2d m_initial;

    public DriveManuallyTest(SwerveDriveSubsystem drivetrain, TestListener listener) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
        m_initial = m_drivetrain.getPose();
    }

    public Twist2d treatment() {
        // the treatment is to move in +x at 0.5 m/s for 2 s so the result should be 1m
        return new Twist2d(0.5, 0, 0);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        // check along the way
        // if(bad) terminate = true
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

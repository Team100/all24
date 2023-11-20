package org.team100.lib.selftest;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveManuallyTest extends Command {
    private static final double kExpectedDuration = 5;
    private SwerveDriveSubsystem m_drivetrain;
    private TestListener m_listener;
    private final Timer m_timer;
    private boolean terminate = false;

    public DriveManuallyTest(SwerveDriveSubsystem drivetrain, TestListener listener) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
    }

    public Twist2d treatment() {
        return new Twist2d(1, 0, 0);
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
        if (MathUtil.isNear(5, time, 0.1))
            m_listener.pass(this, "final time ok: %5.3f", time);
        else
            m_listener.fail(this, "final time bad: %5.3f", time);

        Pose2d state = m_drivetrain.getPose();

        if (MathUtil.isNear(5, state.getX(), 0.1))
            m_listener.pass(this, "final x ok: %5.3f", state.getX());
        else
            m_listener.fail(this, "final x bad: %5.3f", state.getX());

        if (MathUtil.isNear(5, state.getY(), 0.1))
            m_listener.pass(this, "final y ok: %5.3f", state.getY());
        else
            m_listener.fail(this, "final y bad: %5.3f", state.getY());

        if (MathUtil.isNear(5, state.getRotation().getRadians(), 0.1))
            m_listener.pass(this, "final rotation ok: %5.3f", state.getRotation().getRadians());
        else
            m_listener.fail(this, "final rotation bad: %5.3f", state.getRotation().getRadians());
    }

}

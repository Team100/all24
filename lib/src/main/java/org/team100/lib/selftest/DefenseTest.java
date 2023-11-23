package org.team100.lib.selftest;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Verify the "x" pattern.
 */
@ExcludeFromJacocoGeneratedReport
public class DefenseTest extends Command {
    private static final double kExpectedDuration = 0.5;
    private SwerveDriveSubsystem m_drivetrain;
    private TestListener m_listener;
    private final Timer m_timer;
    private boolean terminate = false;

    /**
     * Verify the "x" pattern.
     */
    public DefenseTest(SwerveDriveSubsystem drivetrain, TestListener listener) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public boolean isFinished() {
        return terminate || m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        double time = m_timer.get();
        if (MathUtil.isNear(kExpectedDuration, time, 0.1))
            m_listener.pass(this, "final time ok: %5.3f", time);
        else
            m_listener.fail(this, "final time bad: %5.3f", time);

        SwerveModulePosition[] positions = m_drivetrain.positions();
        if (MathUtil.isNear(Math.PI / 4, positions[0].angle.getRadians(), 0.01))
            m_listener.pass(this, "front left %5.3f", positions[0].angle.getRadians());
        else
            m_listener.fail(this, "front left %5.3f", positions[0].angle.getRadians());

        if (MathUtil.isNear(7 * Math.PI / 4, positions[1].angle.getRadians(), 0.01))
            m_listener.pass(this, "front right %5.3f", positions[1].angle.getRadians());
        else
            m_listener.fail(this, "front right %5.3f", positions[1].angle.getRadians());

        if (MathUtil.isNear(3 * Math.PI / 4, positions[2].angle.getRadians(), 0.01))
            m_listener.pass(this, "rear left %5.3f", positions[2].angle.getRadians());
        else
            m_listener.fail(this, "rear left %5.3f", positions[2].angle.getRadians());

        if (MathUtil.isNear(5 * Math.PI / 4, positions[3].angle.getRadians(), 0.01))
            m_listener.pass(this, "rear right %5.3f", positions[3].angle.getRadians());
        else
            m_listener.fail(this, "rear right %5.3f", positions[3].angle.getRadians());
    }

}

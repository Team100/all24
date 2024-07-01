package org.team100.lib.selftest;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Verify the "x" pattern. The command under test never ends.
 */
@ExcludeFromJacocoGeneratedReport
public class DefenseSelfTest extends Command {
    // this fails below 1s, which is quite a bit too slow
    private static final double kExpectedDuration = 1;
    private static final double kToleranceRad = 0.01;

    private final SwerveDriveSubsystem m_drivetrain;
    private final SelfTestListener m_listener;
    private final Timer m_timer;

    private boolean terminate = false;

    /**
     * Verify the "x" pattern.
     */
    public DefenseSelfTest(SwerveDriveSubsystem drivetrain, SelfTestListener listener) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.restart();
        terminate = false;
    }

    @Override
    public void execute() {
        // check for progress
        SwerveModuleState[] states = m_drivetrain.getSwerveLocal().states();
        if (!MathUtil.isNear(Math.PI / 4, states[0].angle.getRotations(), kToleranceRad)) {
            return;
        }
        if (!MathUtil.isNear(-1 * Math.PI / 4, states[1].angle.getRotations(), kToleranceRad)) {
            return;
        }
        if (!MathUtil.isNear(3 * Math.PI / 4, states[2].angle.getRotations(), kToleranceRad)) {
            return;
        }
        if (!MathUtil.isNear(-3 * Math.PI / 4, states[3].angle.getRotations(), kToleranceRad)) {
            return;
        }
        // we're all done
        terminate = true;
    }

    @Override
    public boolean isFinished() {
        return terminate || m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        double time = m_timer.get();
        String fmt = "final time expected %5.3f actual %5.3f";
        if (MathUtil.isNear(kExpectedDuration, time, 0.1))
            m_listener.pass(this, fmt, kExpectedDuration, time);
        else
            m_listener.fail(this, fmt, kExpectedDuration, time);

        SwerveModulePosition[] positions = m_drivetrain.getSwerveLocal().positions();
        fmt = "front left expected %5.3f actual %5.3f";
        double expected = Math.PI / 4;
        double actual = positions[0].angle.getRadians();
        if (MathUtil.isNear(expected, actual, kToleranceRad))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);

        fmt = "front right expected %5.3f actual %5.3f";
        expected = -1 * Math.PI / 4;
        actual = positions[1].angle.getRadians();
        if (MathUtil.isNear(expected, actual, kToleranceRad))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);

        fmt = "rear left expected %5.3f actual %5.3f";
        expected = 3 * Math.PI / 4;
        actual = positions[2].angle.getRadians();
        if (MathUtil.isNear(expected, actual, kToleranceRad))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);

        fmt = "rear right expected %5.3f actual %5.3f";
        expected = -3 * Math.PI / 4;
        actual = positions[3].angle.getRadians();
        if (MathUtil.isNear(expected, actual, kToleranceRad))
            m_listener.pass(this, fmt, expected, actual);
        else
            m_listener.fail(this, fmt, expected, actual);
    }

}

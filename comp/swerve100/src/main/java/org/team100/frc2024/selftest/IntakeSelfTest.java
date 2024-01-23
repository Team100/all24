package org.team100.frc2024.selftest;

import org.team100.frc2024.motion.intake.Intake;
import org.team100.lib.selftest.SelfTestListener;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Test the Intake subsystem. */
@ExcludeFromJacocoGeneratedReport
public class IntakeSelfTest extends Command {
    private static final double kExpectedDuration = 1;
    private static final double kExpectedVelocity = 1;

    private final Intake m_intake;
    private final SelfTestListener m_listener;
    private final Timer m_timer;

    private double maxVelocity = 0;
    private boolean pass = false;

    public IntakeSelfTest(Intake intake, SelfTestListener listener) {
        m_intake = intake;
        m_listener = listener;
        m_timer = new Timer();
    }

    public void treatment() {
        // spin up the mechanism.
        m_intake.intake();
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        // during the test period, the velocity should exceed the expected velocity.
        double v = m_intake.getVelocity();
        maxVelocity = Math.max(maxVelocity, v);
        if (v > kExpectedVelocity) {
            pass = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        String fmt = "expected speed %5.3f actual %5.3f";
        if (pass) {
            m_listener.pass(this, fmt, kExpectedVelocity, maxVelocity);
        } else {
            m_listener.fail(this, fmt, kExpectedVelocity, maxVelocity);
        }
    }

}

package org.team100.frc2024.selftest;

import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.lib.selftest.SelfTestListener;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Test the Amp subsystem. */
@ExcludeFromJacocoGeneratedReport
public class AmpSelfTest extends Command {
    // this is a sequence
    private static final double kUpTime = 1;
    private static final double kWaitTime = 2;
    private static final double kDownTime = 3;
    private static final double kExpectedPosition = 2;

    private final AmpSubsystem m_amp;
    private final SelfTestListener m_listener;
    private final Timer m_timer;

    private double position = 0;
    private boolean pass = false;

    public AmpSelfTest(AmpSubsystem amp, SelfTestListener listener) {
        m_amp = amp;
        m_listener = listener;
        m_timer = new Timer();
    }

    public void treatment() {
        // move the arm up, wait, and move it down.
        if (m_timer.get() < kUpTime) {
            m_amp.setAmpPosition(2);
        } else if (m_timer.get() > kWaitTime) {
            m_amp.setAmpPosition(0);
        }
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        // the arm should be near the correct position at some
        // point during the waiting period.
        if (m_timer.get() > kUpTime && m_timer.get() < kWaitTime) {
            position = m_amp.getPositionRad();
            if (MathUtil.isNear(position, kExpectedPosition, 0.1)) {
                pass = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > kDownTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_amp.stop();
        String fmt = "expected position %5.3f actual %5.3f";
        if (pass) {
            m_listener.pass(this, fmt, kExpectedPosition, position);
        } else {
            m_listener.fail(this, fmt, kExpectedPosition, position);
        }
    }

}

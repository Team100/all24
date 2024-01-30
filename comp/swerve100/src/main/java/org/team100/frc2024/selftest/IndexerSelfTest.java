package org.team100.frc2024.selftest;

import org.team100.frc2024.motion.indexer.IndexerSubsystem;
import org.team100.lib.selftest.SelfTestListener;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Test the Indexer subsystem. */
@ExcludeFromJacocoGeneratedReport
public class IndexerSelfTest extends Command {
    private static final double kExpectedDuration = 1;
    private static final double kExpectedVelocity = 1;

    private final IndexerSubsystem m_indexer;
    private final SelfTestListener m_listener;
    private final Timer m_timer;

    private double maxVelocity = 0;
    private boolean pass = false;

    public IndexerSelfTest(IndexerSubsystem indexer, SelfTestListener listener) {
        m_indexer = indexer;
        m_listener = listener;
        m_timer = new Timer();
    }
    
    public void treatment() {
        // spin up the mechanism.
        m_indexer.index();
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        // during the test period, the velocity should exceed the expected velocity.
        double v = m_indexer.getVelocity();
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
        m_indexer.stop();
        String fmt = "expected speed %5.3f actual %5.3f";
        if (pass) {
            m_listener.pass(this, fmt, kExpectedVelocity, maxVelocity);
        } else {
            m_listener.fail(this, fmt, kExpectedVelocity, maxVelocity);
        }
    }

}

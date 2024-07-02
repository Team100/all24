package org.team100.frc2024.selftest;

import java.util.OptionalDouble;

import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.selftest.SelfTestListener;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Test the Shooter subsystem. */
@ExcludeFromJacocoGeneratedReport
public class ShooterSelfTest extends Command {
    private static final double kExpectedDuration = 1;
    private static final double kExpectedVelocity = 1;

    private final DrumShooter m_shooter;
    private final SelfTestListener m_listener;
    private final Timer m_timer;

    private double maxVelocity = 0;
    private boolean pass = false;

    public ShooterSelfTest(DrumShooter shooter, SelfTestListener listener) {
        m_shooter = shooter;
        m_listener = listener;
        m_timer = new Timer();
    }

    public void treatment() {
        // spin up the mechanism.
        m_shooter.forward();
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        // during the test period, the velocity should exceed the expected velocity.
        OptionalDouble v = m_shooter.getVelocity();
        if (v.isEmpty())
            return;
        maxVelocity = Math.max(maxVelocity, v.getAsDouble());
        if (v.getAsDouble() > kExpectedVelocity) {
            pass = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        String fmt = "expected speed %5.3f actual %5.3f";
        if (pass) {
            m_listener.pass(this, fmt, kExpectedVelocity, maxVelocity);
        } else {
            m_listener.fail(this, fmt, kExpectedVelocity, maxVelocity);
        }
    }

}

package org.team100.subsystems;

import org.team100.motion.VelocityServoWithFeedback;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: the old code worked differently: it would back up a bit at the end to
 * set the next ball in a standard place. maybe we don't need that?
 */
public class FeedSubsystem extends SubsystemBase {
    private static final double kFeedSpeed = 1.0; // full speed
    private static final double kBackSpeed = -0.5; // back a bit more slowly
    private final VelocityServoWithFeedback m_servo;

    public FeedSubsystem() {
        m_servo = new VelocityServoWithFeedback("feed", 9, 9);
    }

    public Command feed() {
        return run(() -> m_servo.setSpeed(kFeedSpeed));
    }

    public Command back() {
        return run(() -> m_servo.setSpeed(kBackSpeed));
    }

    public Command stop() {
        return run(() -> m_servo.setSpeed(0));
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }
}

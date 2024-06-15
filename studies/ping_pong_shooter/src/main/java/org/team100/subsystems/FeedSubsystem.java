package org.team100.subsystems;

import org.team100.motion.VelocityServoWithFeedback;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: the old code worked differently: it would back up a bit at the end to
 * set the next ball in a standard place. maybe we don't need that?
 * 
 * TODO: add flywheel speed gate here.
 */
public class FeedSubsystem extends SubsystemBase {
    private static final double kFeedSpeed = 0.5; // moderate feed rate
    private static final double kBackSpeed = -0.25; // back a more slowly
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

    /** hold position. */
    public Command stop() {
        return run(() -> m_servo.setSpeed(0));
    }

    public void enable() {
        m_servo.setSpeed(0);
    }

    public void disable() {
        m_servo.disable();
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }
}

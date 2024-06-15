package org.team100.subsystems;

import java.util.function.DoubleSupplier;

import org.team100.motion.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelSubsystem extends SubsystemBase {
    // free speed is about 40? maybe this should be 20.
    // but faster is more impressive so try for maximum speed for now.
    // should use bigger motors/wheels.
    private static final double kMaxSpeedRev_S = 35;

    private final DoubleSupplier m_speed;

    /** Stage right, i.e. facing the direction the balls will go. */
    private final Flywheel m_wheelRight;
    /** Stage left, i.e. facing the direction the balls will go. */
    private final Flywheel m_wheelLeft;

    /** speed controller units, [0,1] */
    public WheelSubsystem(DoubleSupplier speed) {
        m_speed = speed;
        m_wheelRight = new Flywheel("right", 4, Flywheel.Direction.Forward, 0, 1);
        m_wheelLeft = new Flywheel("left", 5, Flywheel.Direction.Reverse, 2, 3);
        disable();
    }

    public Command shoot() {
        return run(() -> setVelocity(kMaxSpeedRev_S * m_speed.getAsDouble()));
    }

    public Command stop() {
        return run(() -> setVelocity(0));
    }

    public void setVelocity(double v) {
        m_wheelRight.setVelocity(v);
        m_wheelLeft.setVelocity(v);
    }

    public void enable() {
        m_wheelRight.enable();
        m_wheelLeft.enable();
    }

    public void disable() {
        m_wheelRight.disable();
        m_wheelLeft.disable();
    }

    @Override
    public void periodic() {
        m_wheelRight.periodic();
        m_wheelLeft.periodic();
    }

}

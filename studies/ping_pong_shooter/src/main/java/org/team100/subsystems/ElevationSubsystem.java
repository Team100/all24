package org.team100.subsystems;

import org.team100.motion.PositionServoWithFeedback;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevationSubsystem extends SubsystemBase {
    private static final double kDtSec = 0.02;
    private static final double kRatePerSec = 0.5;
    // physical limits
    private static final double kMin = 0.15;
    private static final double kMax = 1.00;

    private final PositionServoWithFeedback m_elevation;

    private double m_position;

    public ElevationSubsystem() {
        m_elevation = new PositionServoWithFeedback("angle", 3, 3, kRatePerSec);
    }

    public Command up() {
        return run(() -> setVelocity(kRatePerSec));
    }

    public Command down() {
        return run(() -> setVelocity(-kRatePerSec));
    }

    public Command stop() {
        return run(() -> setVelocity(0));
    }

    public void enable() {
        m_position = m_elevation.getPosition();
        m_elevation.enable();
    }

    public void disable() {
        m_elevation.disable();
    }

    @Override
    public void periodic() {
        m_elevation.periodic();
    }

    ////////////////////////////////////////////////////////

    private void setVelocity(double ratePerSec) {
        m_position += ratePerSec * kDtSec;
        m_position = MathUtil.clamp(m_position, kMin, kMax);
        m_elevation.setPosition(m_position);
    }
}

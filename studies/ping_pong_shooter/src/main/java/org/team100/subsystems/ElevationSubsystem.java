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

    private double m_current_elevation;

    public ElevationSubsystem() {
        m_elevation = new PositionServoWithFeedback("angle", 3, 3, kRatePerSec);
    }

    public void init() {
        m_elevation.init();
        m_current_elevation = m_elevation.get();
    }

    public Command up() {
        return run(this::incrementPosition);
    }

    public Command down() {
        return run(this::decrementPosition);
    }

    public Command stop() {
        return run(this::maintainPosition);
    }

    @Override
    public void periodic() {
        m_elevation.periodic();
    }

    ////////////////////////////////////////////////////////

    private void incrementPosition() {
        m_current_elevation += kRatePerSec* kDtSec;
        m_current_elevation = MathUtil.clamp(m_current_elevation, kMin, kMax);
        m_elevation.set(m_current_elevation);
    }

    private void decrementPosition() {
        m_current_elevation -= kRatePerSec * kDtSec;
        m_current_elevation = MathUtil.clamp(m_current_elevation, kMin, kMax);
        m_elevation.set(m_current_elevation);
    }

    private void maintainPosition() {
        m_elevation.set(m_current_elevation);
    }

}

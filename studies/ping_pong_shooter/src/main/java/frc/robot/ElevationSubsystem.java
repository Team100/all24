package frc.robot;

import org.team100.motion.PositionServoWithFeedback;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevationSubsystem extends SubsystemBase {
    private static final double ELEVATION_SPEED = 0.01;

    private final PositionServoWithFeedback m_elevation;

    private double m_current_elevation;

    public ElevationSubsystem() {
        m_elevation = new PositionServoWithFeedback(3, 3, ELEVATION_SPEED);
        // initialize the profile
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

    public void setPosition(double v) {
        m_elevation.set(v);
    }

    public void incrementPosition() {
        m_current_elevation += ELEVATION_SPEED;
        m_elevation.set(m_current_elevation);
    }

    public void decrementPosition() {
        m_current_elevation -= ELEVATION_SPEED;
        m_elevation.set(m_current_elevation);
    }

    public void maintainPosition() {
        m_elevation.set(m_current_elevation);
    }

}

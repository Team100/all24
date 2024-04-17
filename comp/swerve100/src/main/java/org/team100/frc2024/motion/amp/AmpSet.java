package org.team100.frc2024.motion.amp;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Set the amp pivot angle goal.
 * 
 * This is a class because it resets the control and encoder on init.
 */
public class AmpSet extends Command {
    private final AmpPivot m_pivot;
    private final double m_goal;

    public AmpSet(AmpPivot pivot, double goal) {
        m_pivot = pivot;
        m_goal = goal;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        m_pivot.reset();
    }

    @Override
    public void execute() {
        m_pivot.setAmpPosition(m_goal);
    }
}

package org.team100.frc2024.motion.amp;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.wpilibj2.command.Command;

/** Set the amp pivot angle goal. */
public class AmpSet extends Command implements Glassy {
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

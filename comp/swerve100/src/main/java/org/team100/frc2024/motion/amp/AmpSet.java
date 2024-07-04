package org.team100.frc2024.motion.amp;

import org.team100.lib.commands.Command100;
import org.team100.lib.telemetry.Telemetry.Logger;

/**
 * Set the amp pivot angle goal.
 * 
 * This is a class because it resets the control and encoder on init.
 */
public class AmpSet extends Command100 {
    private final AmpPivot m_pivot;
    private final double m_goal;

    public AmpSet(Logger parent, AmpPivot pivot, double goal) {
        super(parent);
        m_pivot = pivot;
        m_goal = goal;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize100() {
        m_pivot.reset();
    }

    @Override
    public void execute100(double dt) {
        m_pivot.setAmpPosition(m_goal);
    }
}

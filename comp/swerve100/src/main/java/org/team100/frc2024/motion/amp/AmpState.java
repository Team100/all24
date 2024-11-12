package org.team100.frc2024.motion.amp;

import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Model100;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Set the amp pivot state, including current limit, position, and velocity.
 * 
 * These can be used in sequence, to allow varying levels of speed and effort in
 * a single motion: make the goal velocity of the first one equal to the cruise
 * velocity of the second, for example.
 */
public class AmpState extends Command implements Glassy  {
    private final AmpPivot m_pivot;
    private final Model100 m_goal;
    private final Profile100 m_profile;
    private final double m_torqueLimit;
    private final double m_toleranceRad;
    // run forever
    private final boolean m_hold;

    public AmpState(
            AmpPivot pivot,
            Model100 goal,
            Profile100 profile,
            double torqueLimit,
            double toleranceRad,
            boolean hold) {
        m_pivot = pivot;
        m_goal = goal;
        m_profile = profile;
        m_torqueLimit = torqueLimit;
        m_toleranceRad = toleranceRad;
        m_hold = hold;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        if (isFinished()) {
            return;
        }
        m_pivot.setTorqueLimit(m_torqueLimit);
        m_pivot.setProfile(m_profile);
        m_pivot.reset();
    }

    @Override
    public void execute() {
        if (isFinished()) {
            // don't pulse the output if we're going to be "finished"
            return;
        }
        m_pivot.setAmpState(m_goal.control());
    }

    // depend on the SelectCommand to decide when to switch, and
    // hold position forever.
    public boolean done() {
        OptionalDouble opt = m_pivot.getPositionRad();
        if (opt.isEmpty())
            return true;
        double positionRad = opt.getAsDouble();
        if (m_goal.v() > 0) {
            // going up
            return (positionRad > m_goal.x() - m_toleranceRad);
            // already passed the goal
        } else if (m_goal.v() < 0) {
            // going down
            return (positionRad < m_goal.x() + m_toleranceRad);
            // already passed the goal
        } else {
            // goal is stationary, are we close?
            return Math.abs(m_goal.x() - positionRad) < m_toleranceRad;
        }

    }

    @Override
    public boolean isFinished() {
        if (m_hold)
            return false;
        return done();
    }

}

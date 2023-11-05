package org.team100.controllib.reference.examples;

import org.team100.controllib.trajectory.TrapezoidProfile;

/**
 * Generate and follow a feasible trajectory, using WPI trapezoid profiles.
 * This produces an intolerable amount of control ringing at the end of the
 * profile, which I think is due to the infinite jerk there.
 */
public class TrapezoidalReference1D extends Reference1D {
    private final TrapezoidProfile.State m_goal;
    private final TrapezoidProfile.State m_setpoint;
    private final TrapezoidProfile.Constraints m_constraints;
    private final TrapezoidProfile m_profile;

    public TrapezoidalReference1D() {
        m_goal = new TrapezoidProfile.State(1, 0, 0);
        m_setpoint = new TrapezoidProfile.State(0, 0, 0);
        m_constraints = new TrapezoidProfile.Constraints(0.5, 0.5);
        m_profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    }
    
    @Override
   public double position(double timeSec) {
        TrapezoidProfile.State s = m_profile.calculate(timeSec);
        return s.position;
    }

    @Override
    public double velocity(double timeSec) {
        TrapezoidProfile.State s = m_profile.calculate(timeSec);
        return s.velocity;
    }

    @Override
    public double acceleration(double timeSec) {
        TrapezoidProfile.State s = m_profile.calculate(timeSec);
        return s.acceleration;
    }
}

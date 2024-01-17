package org.team100.lib.profile;

import org.team100.lib.controller.State100;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Wrap the WPI profile. */
public class ProfileWPI implements Profile100 {

    TrapezoidProfile m_profile;

    public ProfileWPI(double maxVel, double maxAccel) {
        Constraints constraints = new Constraints(maxVel, maxAccel);
        m_profile = new TrapezoidProfile(constraints);
    }

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        State result = m_profile.calculate(dt, new State(initial.x(), initial.v()), new State(goal.x(), goal.v()));
        return new State100(result.position, result.velocity, 0);
    }
}

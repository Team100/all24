package org.team100.lib.profile;

import org.team100.lib.telemetry.NamedChooser;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains a few motion profiles to choose from.
 */
public class ChoosableProfile {
    public enum Mode {
        /** The WPILib trapezoidal profile with infinite jerk */
        TRAPEZOID,
        /** Preview from 2024, takes motor EMF into account */
        EXPONENTIAL
    }

    private final SendableChooser<Mode> m_chooser;

    // the profile class is both a stateful follower and
    // a stateless calculator. we use the stateless one so we can make
    // the profile object once.
    private final TrapezoidProfile m_trapezoid;

    /** The default is specifiable mostly for testing. */
    public ChoosableProfile(
            double maxVel,
            double maxAccel,
            Mode defaultMode) {
        m_chooser = new NamedChooser<>("Motion Profile");
        for (Mode mode : Mode.values()) {
            m_chooser.addOption(mode.name(), mode);
        }
        m_chooser.setDefaultOption(defaultMode.name(), defaultMode);
        SmartDashboard.putData(m_chooser);

        m_trapezoid = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    public State calculate(double t, State goal, State current) {
        switch (m_chooser.getSelected()) {
            case TRAPEZOID:
                return m_trapezoid.calculate(t, goal, current);
            case EXPONENTIAL:
                // TODO: figure out what these inputs mean.
                ExponentialProfile.Constraints constraints = ExponentialProfile.Constraints.fromStateSpace(1, 1, 1);
                ExponentialProfile eprofile = new ExponentialProfile(constraints);
                ExponentialProfile.State estate = eprofile.calculate(t,
                        new ExponentialProfile.State(goal.position, goal.velocity),
                        new ExponentialProfile.State(current.position, current.velocity));
                return new State(estate.position, estate.velocity);
            default:
                return new State();
        }
    }

    public Mode getSelected() {
        return m_chooser.getSelected();
    }

}

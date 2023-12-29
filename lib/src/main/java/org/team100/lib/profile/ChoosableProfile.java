package org.team100.lib.profile;

import org.team100.lib.telemetry.ProfileModeChooser;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains a few motion profiles to choose from.
 */
public class ChoosableProfile {
    public enum Mode {
        /** The WPILib trapezoidal profile with infinite jerk */
        TRAPEZOID,
        /**
         * New for 2024, takes motor EMF into account.
         * This should be considered experimental.
         */
        EXPONENTIAL
    }

    private final SendableChooser<Mode> m_chooser;

    // the profile class is both a stateful follower and
    // a stateless calculator. we use the stateless one so we can make
    // the profile object once.
    private final TrapezoidProfile100 m_trapezoid;
    private final ExponentialProfile eprofile;

    /** The default is specifiable mostly for testing. */
    public ChoosableProfile(
            double maxVel,
            double maxAccel,
            Mode defaultMode) {
        m_chooser = ProfileModeChooser.get("Motion Profile");
        for (Mode mode : Mode.values()) {
            m_chooser.addOption(mode.name(), mode);
        }
        m_chooser.setDefaultOption(defaultMode.name(), defaultMode);
        SmartDashboard.putData(m_chooser);

        m_trapezoid = new TrapezoidProfile100(
                new TrapezoidProfile100.Constraints(maxVel, maxAccel));
                
        // These constraints are completely made up.
        // If you want to try the exponential profile, you'll have to
        // calibrate them
        eprofile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        10, 10, 5));
    }

    public TrapezoidProfile100.State calculate(double t, TrapezoidProfile100.State goal, TrapezoidProfile100.State current) {
        switch (m_chooser.getSelected()) {
            case TRAPEZOID:
                return m_trapezoid.calculate(t, goal, current);
            case EXPONENTIAL:
                ExponentialProfile.State estate = eprofile.calculate(t,
                        new ExponentialProfile.State(current.position, current.velocity),
                        new ExponentialProfile.State(goal.position, goal.velocity));
                return new TrapezoidProfile100.State(estate.position, estate.velocity);
            default:
                return new TrapezoidProfile100.State();
        }
    }

    public Mode getSelected() {
        return m_chooser.getSelected();
    }

}

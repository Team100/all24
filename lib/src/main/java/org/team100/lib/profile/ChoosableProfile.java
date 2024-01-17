package org.team100.lib.profile;

import org.team100.lib.controller.State100;
import org.team100.lib.telemetry.ProfileModeChooser;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains a few motion profiles to choose from.
 */
public class ChoosableProfile implements Profile100 {
    public enum Mode {
        /** The WPILib trapezoidal profile with infinite jerk */
        PROFILE_WPI,
        /** The Team 100 profile, which handles some cases the WPI one does not. */
        PROFILE_100,
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
    private final ProfileWPI m_profileWPI;
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
        
        m_profileWPI = new ProfileWPI(maxVel, maxAccel);

        m_trapezoid = new TrapezoidProfile100(
                new Constraints100(maxVel, maxAccel), 0.05);

        // These constraints are completely made up.
        // If you want to try the exponential profile, you'll have to
        // calibrate them
        eprofile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        10, 10, 5));
    }

    /** Note order here, initial first, goal second. */
    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        switch (m_chooser.getSelected()) {
            case PROFILE_WPI:
                return m_profileWPI.calculate(dt, initial, goal);
            case PROFILE_100:
                return m_trapezoid.calculate(dt, initial, goal);
            case EXPONENTIAL:
                ExponentialProfile.State estate = eprofile.calculate(dt,
                        new ExponentialProfile.State(initial.x(), initial.v()),
                        new ExponentialProfile.State(goal.x(), goal.v()));
                return new State100(estate.position, estate.velocity);
            default:
                return new State100(0, 0);
        }
    }

    Mode getSelected() {
        return m_chooser.getSelected();
    }

}

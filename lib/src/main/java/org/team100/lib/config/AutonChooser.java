package org.team100.lib.config;

import org.team100.lib.telemetry.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A glass widget for choosing auton routines.
 */
public class AutonChooser {
    public enum Routine {
        FIVENOTE,
        COMPLEMENTARY;
    }

    private static final SendableChooser<Routine> m_routineChooser = new NamedChooser<>("Auton Routine") {
    };

    static {
        for (Routine routine : Routine.values()) {
            m_routineChooser.addOption(routine.name(), routine);
        }
        m_routineChooser.setDefaultOption(Routine.FIVENOTE.name(), Routine.FIVENOTE);
        SmartDashboard.putData(m_routineChooser);
    }

    public static Routine routine() {
        return m_routineChooser.getSelected();
    }

    private AutonChooser() {
        //
    }

}

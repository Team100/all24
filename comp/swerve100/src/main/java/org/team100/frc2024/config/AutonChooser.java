package org.team100.frc2024.config;

import org.team100.frc2024.commands.AutonCommand;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A glass widget for choosing auton routines.  Used by {@link AutonCommand}.
 */
public class AutonChooser {
    public enum Routine {
        FOUR_NOTE,
        FIVE_NOTE,
        COMPLEMENTARY,
        COMPLEMENTARY2,
        SIBLING,
        NOTHING;
    }

    private static final SendableChooser<Routine> m_routineChooser = new NamedChooser<>("Auton Routine") {
    };

    static {
        for (Routine routine : Routine.values()) {
            m_routineChooser.addOption(routine.name(), routine);
        }
        m_routineChooser.setDefaultOption(Routine.FIVE_NOTE.name(), Routine.FIVE_NOTE);
        SmartDashboard.putData(m_routineChooser);
    }

    /** The selected routine.  Used by AutonCommand for selection. */
    public static Routine routine() {
        return m_routineChooser.getSelected();
    }

    private AutonChooser() {
        //
    }

}

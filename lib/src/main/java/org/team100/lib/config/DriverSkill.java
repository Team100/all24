package org.team100.lib.config;

import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Driver skill level is used to limit speed.  Advanced drivers get full
 * speed; beginner and intermediate drivers get less.
 * 
 * This is also a safety mechanism: in development, the "normal" speed should
 * be much lower than full speed.
 */
public class DriverSkill {
    public enum Level {
        BEGINNER(0.25),
        INTERMEDIATE(0.5),
        ADVANCED(1.0);

        private double scale;

        private Level(double scale) {
            this.scale = scale;
        }

        public double scale() {
            return scale;
        }
    }

    private static final SendableChooser<Level> m_skillChooser = new NamedChooser<>("Driver Skill Level") {
    };

    static {
        for (Level level : Level.values()) {
            m_skillChooser.addOption(level.name(), level);
        }
        m_skillChooser.setDefaultOption(Level.ADVANCED.name(), Level.ADVANCED);
        SmartDashboard.putData(m_skillChooser);
    }

    public static Level level() {
        return m_skillChooser.getSelected();
    }

    private DriverSkill() {
        //
    }
}

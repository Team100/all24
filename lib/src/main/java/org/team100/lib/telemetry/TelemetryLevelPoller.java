package org.team100.lib.telemetry;

import org.team100.lib.async.Async;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Asynchronously looks for updates in telemetry level. */
public class TelemetryLevelPoller {
    private final Telemetry t = Telemetry.get();
    private final SendableChooser<Level> m_levelChooser;

    /** Create a chooser with no default; use setDefault if you want one. */
    public TelemetryLevelPoller(Async async) {
        m_levelChooser = TelemetryLevelChooser.get();
        for (Level level : Level.values()) {
            m_levelChooser.addOption(level.name(), level);
        }
        SmartDashboard.putData(m_levelChooser);
        updateLevel();
        async.addPeriodic(this::updateLevel, 1, "Telemetry");
    }

    /**
     * Set the chooser default option. Note a client like glass will override this
     * default immediately.
     */
    public void setDefault(Level level) {
        if (level.admit(Level.DEBUG)) {
            Util.warn(String.format(
                    "Setting default telemetry to %s.  Comp should use COMP.",
                    level.name()));
        }
        m_levelChooser.setDefaultOption(level.name(), level);
    }

    private void updateLevel() {
        Level selected = m_levelChooser.getSelected();
        if (selected == null)
            return;
        t.setLevel(selected);
    }
}

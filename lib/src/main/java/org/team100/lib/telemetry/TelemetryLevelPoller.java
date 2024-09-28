package org.team100.lib.telemetry;

import java.util.function.Consumer;

import org.team100.lib.async.Async;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Asynchronously looks for updates in telemetry level. */
public class TelemetryLevelPoller {
    private final SendableChooser<Level> m_levelChooser;
    private final Consumer<Level> m_consumer;

    public TelemetryLevelPoller(Async async, Consumer<Level> consumer, Level defaultLevel) {
        m_consumer = consumer;
        m_levelChooser = TelemetryLevelChooser.get();
        for (Level level : Level.values()) {
            m_levelChooser.addOption(level.name(), level);
        }
        m_levelChooser.setDefaultOption(defaultLevel.name(), defaultLevel);
        SmartDashboard.putData(m_levelChooser);
        updateLevel();
        async.addPeriodic(this::updateLevel, 1, "Telemetry");
    }

    public Level getLevel() {
        return m_levelChooser.getSelected();
    }

    private void updateLevel() {
        Level selected = m_levelChooser.getSelected();
        if (selected == null)
            return;
        m_consumer.accept(selected);
    }
}

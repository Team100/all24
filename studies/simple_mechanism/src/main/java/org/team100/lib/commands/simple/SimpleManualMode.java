package org.team100.lib.commands.simple;

import java.util.function.Supplier;

import org.team100.lib.telemetry.SimpleManualModeChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Chooser for single degree of freedom manual control.
 */
public class SimpleManualMode implements Supplier<SimpleManualMode.Mode> {
    public enum Mode {
        VELOCITY,
        POSITION
    }

    private final SendableChooser<Mode> m_manualModeChooser;

    public SimpleManualMode() {
        m_manualModeChooser = SimpleManualModeChooser.get("Simple Manual Mode");
        for (Mode mode : Mode.values()) {
            m_manualModeChooser.addOption(mode.name(), mode);
        }
        m_manualModeChooser.setDefaultOption(
                Mode.VELOCITY.name(),
                Mode.VELOCITY);
        SmartDashboard.putData(m_manualModeChooser);
    }

    public Mode getSelected() {
        return m_manualModeChooser.getSelected();
    }

    @Override
    public Mode get() {
        return getSelected();
    }

}

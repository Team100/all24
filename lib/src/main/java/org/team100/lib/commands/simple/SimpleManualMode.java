package org.team100.lib.commands.simple;

import java.util.function.Supplier;

import org.team100.lib.telemetry.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleManualMode implements Supplier<SimpleManualMode.Mode> {
    public enum Mode {
        DUTY_CYCLE,
        VELOCITY
    }

    private final SendableChooser<Mode> m_manualModeChooser;

    public SimpleManualMode() {
        m_manualModeChooser = new NamedChooser<>("Simple Manual Mode");
        for (Mode mode : Mode.values()) {
            m_manualModeChooser.addOption(mode.name(), mode);
        }
        m_manualModeChooser.setDefaultOption(
                Mode.DUTY_CYCLE.name(),
                Mode.DUTY_CYCLE);
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

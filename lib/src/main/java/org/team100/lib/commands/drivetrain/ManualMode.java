package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualMode implements Supplier<ManualMode.Mode> {
    public enum Mode {
        MODULE_STATE,
        ROBOT_RELATIVE_CHASSIS_SPEED,
        FIELD_RELATIVE_TWIST
    }
    private final SendableChooser<Mode> m_manualModeChooser;

    public ManualMode() {
        m_manualModeChooser = new SendableChooser<>();
        for (Mode mode : Mode.values()) {
            m_manualModeChooser.addOption(mode.name(), mode);
        }
        m_manualModeChooser.setDefaultOption(
                Mode.FIELD_RELATIVE_TWIST.name(),
                Mode.FIELD_RELATIVE_TWIST);
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

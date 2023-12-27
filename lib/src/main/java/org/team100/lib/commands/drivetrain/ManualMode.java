package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.telemetry.ManualModeChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Modes for manual drivetrain control.
 */
public class ManualMode implements Supplier<ManualMode.Mode> {
    public enum Mode {
        /** Control module speed and direction directly */
        MODULE_STATE,
        /** Robot-relative dx, dy, and omega */
        ROBOT_RELATIVE_CHASSIS_SPEED,
        /** Field-relative dx, dy, and omega */
        FIELD_RELATIVE_TWIST,
        /** Field-relative dx and dy, rotational feedback control */
        SNAPS,
        /** Field-relative dx and dy, rotational target lock */
        LOCKED
    }
    private final SendableChooser<Mode> m_manualModeChooser;

    public ManualMode() {
        m_manualModeChooser = ManualModeChooser.get("Manual Drive Mode");
        for (Mode mode : Mode.values()) {
            m_manualModeChooser.addOption(mode.name(), mode);
        }
        m_manualModeChooser.setDefaultOption(
                Mode.SNAPS.name(),
                Mode.SNAPS);
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

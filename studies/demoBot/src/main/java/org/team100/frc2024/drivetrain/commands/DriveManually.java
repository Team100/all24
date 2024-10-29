package org.team100.frc2024.drivetrain.commands;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;

import org.team100.frc2024.drivetrain.TankDriveSubsystem;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual drivetrain control.
 * 
 * Provides four manual control modes:
 * 
 * -- raw module state
 * -- robot-relative
 * -- field-relative
 * -- field-relative with rotation control
 * 
 * Use the mode supplier to choose which mode to use, e.g. using a Sendable
 * Chooser.
 */
public class DriveManually extends Command implements Glassy {
 
    private static final SendableChooser<String> m_manualModeChooser = new NamedChooser<>("Manual Drive Mode") {
    };

    private Supplier<String> m_mode;
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final TankDriveSubsystem m_drive;
    private final Map<String, TankDriver> m_drivers;
    private final TankDriver m_defaultDriver;
    String currentManualMode = null;

    public DriveManually(
            Supplier<DriverControl.Velocity> twistSupplier,
            TankDriveSubsystem robotDrive) {
        m_mode = m_manualModeChooser::getSelected;
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        m_defaultDriver = stop();
        m_drivers = new ConcurrentHashMap<>();
        SmartDashboard.putData(m_manualModeChooser);
        addRequirements(m_drive);
        addName("Manual", true);
        m_drivers.put(
            "Manual",
                new TankDriver() {
                    public void apply(DriverControl.Velocity t) {
                        m_drive.set(t.x(), t.y());
                    }
        });
    }

    @Override
    public void execute() {
        String manualMode = m_mode.get();
        if (manualMode == null) {
            return;
        }

        if (!(manualMode.equals(currentManualMode))) {
            currentManualMode = manualMode;
        }

        // input in [-1,1] control units
        DriverControl.Velocity input = m_twistSupplier.get();
        TankDriver d = m_drivers.getOrDefault(manualMode, m_defaultDriver);
        d.apply(input);

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    /**
     * Override the TankDriver  mode.
     * 
     * For testing only.
     */
    public void overrideMode(Supplier<String> mode) {
        m_mode = mode;
    }

    //////////////

    private TankDriver stop() {
        return new TankDriver() {
            public void apply(DriverControl.Velocity t) {
                m_drive.stop();
            }
        };
    }

    private void addName(String name, boolean isDefault) {
        m_manualModeChooser.addOption(name, name);
        if (isDefault)
            m_manualModeChooser.setDefaultOption(name, name);
    }
}

package org.team100.frc2024.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.frc2024.commands.drivetrain.manual.ManualWithShooterLock;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterLockCommand extends Command {
    private final ManualWithShooterLock m_driver;
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;

    public ShooterLockCommand(
            ManualWithShooterLock driver,
            Supplier<DriverControl.Velocity> twistSupplier,
            SwerveDriveSubsystem drive) {
        m_driver = driver;
        m_twistSupplier = twistSupplier;
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_driver.reset(m_drive.getState());
    }

    @Override
    public void execute() {
        FieldRelativeVelocity twist = m_driver.apply(m_drive.getState(), m_twistSupplier.get());
        m_drive.driveInFieldCoords(twist);
    }
}

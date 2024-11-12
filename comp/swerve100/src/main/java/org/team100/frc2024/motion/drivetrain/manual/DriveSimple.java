package org.team100.frc2024.motion.drivetrain.manual;

import org.team100.frc2024.commands.drivetrain.manual.ManualWithShooterLock;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveSimple extends Command {

    private final ManualWithShooterLock m_driver;
    private final SwerveDriveSubsystem m_swerve;

    public DriveSimple(SwerveDriveSubsystem swerve, ManualWithShooterLock driver) {
        m_driver = driver;
        m_swerve = swerve;
    }

    @Override
    public void initialize() {
        m_driver.reset(m_swerve.getState());
    }

    @Override
    public void execute() {
        FieldRelativeVelocity twist = m_driver.apply(m_swerve.getState(), new DriverControl.Velocity(0, 0, 0));
        m_swerve.driveInFieldCoords(twist);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return m_driver.isAligned();
    }
}

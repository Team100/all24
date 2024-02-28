package org.team100.frc2024.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

//TODO crappy implementation of this just for EPA, will change this - Sanjan

public class ShooterLockCommand extends Command {

    ManualWithShooterLock m_driver;
    Supplier<Twist2d> m_twistSupplier;
    SwerveDriveSubsystem m_drive;

    public ShooterLockCommand(ManualWithShooterLock driver, Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem drive) {

        m_driver = driver;
        m_twistSupplier = twistSupplier;
        m_drive = drive;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_driver.reset(m_drive.getPose());
    }

    @Override
    public void execute() {
        System.out.println(m_twistSupplier.get());
        Twist2d twist = m_driver.apply(m_drive.getState(), m_twistSupplier.get());

        m_drive.driveInFieldCoords(twist, 0.02);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

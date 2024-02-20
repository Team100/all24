package org.team100.frc2024.motion.drivetrain.manual;


import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveSimple extends Command {
  /** Creates a new DriveSimple. */
  
  ManualWithShooterLock m_driver;
  SwerveDriveSubsystem m_swerve;
  public DriveSimple(SwerveDriveSubsystem swerve, ManualWithShooterLock driver) {
    m_driver = driver;
    m_swerve = swerve;
  }

  @Override
  public void initialize() {
    m_driver.reset(m_swerve.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Twist2d twist = m_driver.apply(m_swerve.getState(), new Twist2d(0, 0, 0));
    m_swerve.driveInFieldCoords(twist, 0.02);
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

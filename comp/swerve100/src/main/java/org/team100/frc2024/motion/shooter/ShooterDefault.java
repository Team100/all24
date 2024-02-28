// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.shooter;

import java.util.function.Supplier;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterDefault extends Command {
  /** Creates a new ShooterDefault. */

  double kThreshold = 8;
  Shooter m_shooter;
  SwerveDriveSubsystem m_drive;
  Supplier<Double> m_pivotUpSupplier;
  Supplier<Double> m_pivotDownSupplier;

  public ShooterDefault(Shooter shooter, SwerveDriveSubsystem drive,  Supplier<Double> pivotUpSupplier, Supplier<Double> pivotDownSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_drive = drive;
    m_pivotUpSupplier = pivotUpSupplier;
    m_pivotDownSupplier = pivotDownSupplier;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = m_drive.getPose().getTranslation().getDistance(ShooterUtil.getSpeakerTranslation());
    System.out.println(distance);
    switch(RobotState100.getRobotState()){
        case SHOOTING:

            switch(RobotState100.getShooterState()){
                case DEFAULTSHOOT:
                    m_shooter.forward();
                    // m_shooter.setAngleWithOverride(ShooterUtil.getAngle(m_drive.getPose().getX()), m_pivotUpSupplier.get(), m_pivotDownSupplier.get()); //22.5a   
                    m_shooter.setAngle(ShooterUtil.getAngle(distance)); //22.5a   
                    System.out.println("DEFAULT SHOOOOT");
                    break;
                case STOP:
                    m_shooter.stop();
                    break;
                default:
                    m_shooter.setDutyCycle(0);
                    break;

            }
            break;
            
        case AMPING:
            // m_shooter.stop();
            // System.out.println("AMPIIING");
            break;
        case NONE:
            // m_shooter.stop();
            break;
        default:
            break;
            // m_shooter.stop();
            // m_shooter.setAngle(0.0);

            
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

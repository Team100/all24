// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterDefault extends Command {
  /** Creates a new ShooterDefault. */

  double kThreshold = 8;
  Shooter m_shooter;
  SwerveDriveSubsystem m_drive;

  public ShooterDefault(Shooter shooter, SwerveDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_drive = drive;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(RobotState100.getRobotState()){
        case SHOOTING:
            // switch(RobotState100.getShooterState()){
            //     case DEFAULTSHOOT:
            //         m_shooter.pivotAndRamp(m_drive, kThreshold);
            //         break;
            //     case FEED:
            //         m_shooter.feed();
            //         break;
            //     case READYTOSHOOT:
            //         m_shooter.pivotAndRamp(m_drive, kThreshold); // pivots and ramps before anything
            //         if(m_shooter.readyToShoot(m_drive)){ //if angle and rotation is good enough
            //             m_shooter.feed(); //if velocity is good enough then shoot it
            //         }
            //         break;
            //     case DOWN:
            //         m_shooter.setAngle(0.0);
            //         break;
            //     default:
            //         m_shooter.pivotAndRamp(m_drive, kThreshold);
            //         break;

            // }
            // System.out.println("SHOOOOT");
            // m_shooter.setAngle(22.0);
            switch(RobotState100.getShooterState()){
                case DEFAULTSHOOT:
                    // System.out.println("AHHHHHHHHH");
                    m_shooter.forward();
                    m_shooter.setAngle(ShooterUtil.getAngle(m_drive.getPose().getX())); //22.5a   
                    break;
                case STOP:
                    // System.out.println("NOOOOOOOOOOOOOOOOOOO");
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

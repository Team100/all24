package org.team100.frc2023.autonomous;

import edu.wpi.first.wpilibj.Timer;

import org.team100.frc2023.commands.DriveScaled;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class DeadDrivetrain extends Command {
  /** Creates a new DeadDrivetrain. */

  Drivetrain m_robotDrive;
  Timer m_timer;
  boolean done = false;
  public DeadDrivetrain(Drivetrain robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    m_timer = new Timer();
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("dead drivetrain init");


    m_robotDrive.removeDefaultCommand();
    m_robotDrive.setDefaultCommand(new DriveScaled(null, m_robotDrive, null));
    done = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("dead drivetrain execute");


    // System.out.println(done);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("dead drivetrain end");

    //
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
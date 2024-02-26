// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.IntakeRoller;
import org.team100.frc2024.motion.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCommand extends Command {
  /** Creates a new FeedCommand. */
  Intake m_intake;
  Shooter m_shooter;
  AmpSubsystem m_amp;
  FeederSubsystem m_feeder;

  double value = 5;

  public OuttakeCommand(Intake intake, Shooter shooter, AmpSubsystem amp, FeederSubsystem feeder) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_intake = intake;
    m_shooter = shooter;
    m_amp = amp;
    m_feeder = feeder;

    addRequirements(m_amp, m_intake, m_shooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        m_feeder.starve();
        m_intake.outtake();
        m_shooter.feed();
        m_amp.driveFeeder(-1);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_amp.driveFeeder(0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

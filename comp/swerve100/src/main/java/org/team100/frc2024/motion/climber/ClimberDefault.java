// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.climber;

import java.util.function.Supplier;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command {
    private static final Telemetry t = Telemetry.get();

  /** Creates a new ClimberDefault. */

  Supplier<Double> m_leftSupplier;
  Supplier<Double> m_rightSupplier;
  Supplier<Boolean> m_overrideSupplier;

  ClimberSubsystem m_climber;
  public ClimberDefault(ClimberSubsystem climber, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier, Supplier<Boolean> overideSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;
    m_climber = climber;
    m_overrideSupplier = overideSupplier;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t.log(Level.DEBUG, "ClimberDefault", "command state", "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t.log(Level.DEBUG, "ClimberDefault", "command state", "execute");

        m_climber.setLeft(m_leftSupplier.get());
        m_climber.setRight(m_rightSupplier.get());
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        t.log(Level.DEBUG, "ClimberDefault", "command state", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

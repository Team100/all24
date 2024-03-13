// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import java.util.Optional;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSmart extends Command {
  private static final Telemetry t = Telemetry.get();

  /** Creates a new ShootSmart. */

  SensorInterface m_sensor;
  FeederSubsystem m_feeder;
  Shooter m_shooter;
  Intake m_intake;
  Timer m_timer;
  boolean atVelocity = false;
  boolean finished = false;
  SwerveDriveSubsystem m_drive;
  boolean m_isPreload;
double m_pivotOverride;

  public ShootSmart(SensorInterface sensor, Shooter shooter, Intake intake, FeederSubsystem feeder, SwerveDriveSubsystem drive, double pivotOverride, boolean isPreload) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_sensor = sensor;
    m_feeder = feeder;
    m_shooter = shooter;
    m_timer = new Timer();
    m_drive = drive;
    m_pivotOverride = pivotOverride;
    m_isPreload = isPreload;

    addRequirements(m_intake, m_feeder, m_shooter);
  }

  public ShootSmart(SensorInterface sensor, Shooter shooter, Intake intake, FeederSubsystem feeder, SwerveDriveSubsystem drive, boolean isPreload) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_sensor = sensor;
    m_feeder = feeder;
    m_shooter = shooter;
    m_timer = new Timer();
    m_drive = drive;
    m_pivotOverride = -1;
    m_isPreload = isPreload;

    addRequirements(m_intake, m_feeder, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance(); 
    if (!alliance.isPresent()) return;

        t.log(Level.DEBUG, "ShootSmart", "command state", "initialize");

double targetShooterAngle;

    double distance = m_drive.getPose().getTranslation().getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
    m_timer.reset();
    m_shooter.forward();
// if(m_pivotOverride == -1){
    m_shooter.setAngle(ShooterUtil.getAngle(distance));
// } else {
    //     m_shooter.setAngle(m_pivotOverride);
    // }
    m_intake.intake();
    m_feeder.feed();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t.log(Level.DEBUG, "ShootSmart", "command state", "execute");
    Optional<Alliance> alliance = DriverStation.getAlliance(); 
    if (!alliance.isPresent()) return;
    
    double angle;
    if(m_pivotOverride == -1){
        double distance = m_drive.getPose().getTranslation().getDistance(ShooterUtil.getSpeakerTranslation(alliance.get())); 
       angle = ShooterUtil.getAngle(distance);
    } else {
        angle = m_pivotOverride;
    }
    // double angle = m_drive.getPose().getTranslation().getDistance(ShooterUtil.getSpeakerTranslation());

    
    if(m_pivotOverride == -1){
        m_shooter.setAngle(angle);
    } else {
        m_shooter.setAngle(angle);
    }

    if(!m_sensor.getFeederSensor()){

      m_intake.stop();
      m_feeder.stop();

      if(m_shooter.atVelocitySetpoint(m_isPreload)){
        // if(Math.abs(m_shooter.getPivotgPosition() - ShooterUtil.getAngle(m_drive.getPose().getX())) < 1 ){
            atVelocity = true;
            m_timer.start();
          //   }
        } 
    }

    if(atVelocity){

      m_feeder.feed();
      m_intake.intake();

      if(m_timer.get() > 0.2){
        finished = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        t.log(Level.DEBUG, "ShootSmart", "command state", "end");

    atVelocity = false;
    finished = false;
    m_timer.stop();
    m_shooter.stop();
    m_intake.stop();
    m_feeder.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
    // return false;
  }
}

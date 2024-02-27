// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import org.team100.frc2024.Robot;
import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.ShooterState100;
import org.team100.frc2024.RobotState100.State100;
import org.team100.frc2024.motion.intake.IntakeRoller;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private final String m_name;
  private final Telemetry t;
  private final LimitedVelocityServo<Distance100> feedRoller;
  private final double kFeederVelocityM_S = 30;

  public FeederSubsystem(int feederID) {
    int feederLimit = 40;

    m_name = Names.name(this);
    SysParam feederParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);
    t = Telemetry.get();

     switch (Identity.instance) {
            case COMP_BOT:
            //TODO tune kV
                feedRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name +"/Feeder", 
                        feederID, 
                        true, 
                        feederLimit,
                        feederParams, 
                        new FeedforwardConstants(0.122,0,0.1,0.065),
                        new PIDConstants(0.1, 0, 0));
                break;
            case BLANK:
            default:
                feedRoller = ServoFactory.limitedSimulatedVelocityServo(
                    m_name + "/Feed",
                    feederParams);
        }

  }

  public void starve(){
    feedRoller.setDutyCycle(-1);
  }


  public void feed(){
    feedRoller.setDutyCycle(0.8);
    
  }

  public void intake(){
    feedRoller.setDutyCycle(0.1);
    
  }

  public void outtake(){
    feedRoller.setDutyCycle(-0.1);
    
  }

  public void stop(){
    // System.out.println("STOPING FEED" + Timer.getFPGATimestamp());

    feedRoller.setDutyCycle(0);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feedRoller.periodic();
  }
}

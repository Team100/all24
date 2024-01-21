// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.amp;

import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase {
  /** Creates a new AmpSubsystem. */

  private final String m_name;
  SysParam ampAngleParameter;

  private final PositionServo<Angle100> ampAngleMotorLeft;
  private final PositionServo<Angle100> ampAngleMotorRight;

  public AmpSubsystem(int leftPivotID, int rightPivotID) {
      m_name = Names.name(this);
      ampAngleParameter = SysParam.neoPositionServoSystem(15, 8, 20);    

      ampAngleMotorLeft = ServoFactory.neoPositionServo(
            m_name, 
            leftPivotID, 
            false, 
            ampAngleParameter, 
            new PIDController(1, 0, 0));
    
      ampAngleMotorRight = ServoFactory.neoPositionServo(
            m_name, 
            rightPivotID, 
            false, 
            ampAngleParameter, 
            new PIDController(1, 0, 0));
  }

  public void setAmpPosition(double value, double value2) {
    ampAngleMotorRight.setPosition(value);
    ampAngleMotorLeft.setPosition(value);

  }

  public void setAmpVelocity(double value) {
    ampAngleMotorRight.setDutyCycle(value);
    ampAngleMotorLeft.setDutyCycle(value);
  }

  public double getLeftAmpPosition(){
    return ampAngleMotorLeft.getPosition();
  }

  public double getRightAmpPosition(){
    return ampAngleMotorRight.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

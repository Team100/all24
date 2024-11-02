// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.servo.LinearPositionServo;
import org.team100.lib.motion.servo.OnboardLinearDutyCyclePositionServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoTest extends SubsystemBase {

  private final LinearPositionServo servo;
  /** Creates a new NeoTest. */
  public NeoTest(LoggerFactory parent) {
    LoggerFactory child = parent.child(this);
    LoggerFactory rightLogger = child.child("right");
    m_right = comp(rightLogger, rightClimberID,MotorPhase.FORWARD);

    servo = new OnboardLinearDutyCyclePositionServo(
                child.child("left"),
                m_right,
                new PIDControlleservor(0.1, 0, 0),
                new TrapezoidProfile100(0.02, 0.1, 0.01));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

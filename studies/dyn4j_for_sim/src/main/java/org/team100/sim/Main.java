package org.team100.sim;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

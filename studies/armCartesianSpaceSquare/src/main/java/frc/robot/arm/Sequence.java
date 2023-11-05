package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmSubsystem;

public class Sequence extends SequentialCommandGroup {
    private final ArmSubsystem m_robot;
    public Sequence(ArmSubsystem robot) {
        m_robot = robot;
        addCommands(new ArmTrajectory(m_robot, new Translation2d(.6, .6)), 
        new ArmTrajectory(m_robot, new Translation2d(1, .6)),
        new ArmTrajectory(m_robot, new Translation2d(1, 1)),
        new ArmTrajectory(m_robot, new Translation2d(.6, 1)),
        new ArmTrajectory(m_robot, new Translation2d(.6, .6))
        );
    }
}

package frc.robot.armCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subSystems.ArmSubsystem;

public class Sequence extends SequentialCommandGroup {
    private final ArmSubsystem m_armSubsystem;
    public Sequence(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addCommands(new ArmTrajectoryCommand(m_armSubsystem, new Translation2d(.6, .6)), 
        new ArmTrajectoryCommand(m_armSubsystem, new Translation2d(1, .6)),
        new ArmTrajectoryCommand(m_armSubsystem, new Translation2d(1, 1)),
        new ArmTrajectoryCommand(m_armSubsystem, new Translation2d(.6, 1)),
        new ArmTrajectoryCommand(m_armSubsystem, new Translation2d(.6, .6))
        );
    }
}

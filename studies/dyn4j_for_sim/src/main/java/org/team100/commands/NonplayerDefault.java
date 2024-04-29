package org.team100.commands;

import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {

    private final RobotAssembly m_robot;
    public NonplayerDefault(RobotAssembly robot) {
        m_robot = robot;
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public void execute() {
        Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity());
        Tactics.avoidEdges(m_robot.getPose());
        Tactics.avoidSubwoofers(m_robot.getPose());
        Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings());
        Tactics.robotRepulsion(m_robot.getPose(),  m_robot.recentSightings());
    }
}

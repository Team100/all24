package org.team100.commands;

import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private final Tactics m_tactics;

    public NonplayerDefault(RobotAssembly robot) {
        m_tactics = new Tactics(robot);
        addRequirements(robot.getRobotSubsystem());
    }

    @Override
    public void execute() {
        m_tactics.avoidObstacles();
        m_tactics.avoidEdges();
        m_tactics.avoidSubwoofers();
        m_tactics.steerAroundRobots();
        m_tactics.robotRepulsion();
    }
}

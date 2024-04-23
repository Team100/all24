package org.team100.commands;

import org.team100.robot.RobotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private final Tactics m_tactics;

    public NonplayerDefault(RobotSubsystem robot) {
        m_tactics = new Tactics(robot);
        addRequirements(robot);
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

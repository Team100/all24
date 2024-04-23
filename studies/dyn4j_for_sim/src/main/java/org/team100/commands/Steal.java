package org.team100.commands;

import org.team100.robot.RobotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** Drive towards the nearest note and take it. */
public class Steal extends Command {

    private final Alliance m_alliance;
    private final RobotSubsystem m_robot;
    private final Tactics m_tactics;

    public Steal(Alliance alliance, RobotSubsystem robot) {
        m_alliance = alliance;
        m_robot = robot;
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
        goToGoal();
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    private void goToGoal() {
        //
    }

}

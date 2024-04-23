package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.robot.RobotSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PickFromSource extends Command {

    private final Alliance m_alliance;
    private final RobotSubsystem m_robot;
    private final Tactics m_tactics;

    public PickFromSource(Alliance alliance, RobotSubsystem robot) {
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
        Pose2d pose = m_robot.getPose();
        Vector2 position = new Vector2(pose.getX(), pose.getY());
        Vector2 positionError = position.to(m_robot.sourcePosition());
        // TODO: better pick geometry
        // double angle = pose.getRotation().getRadians();
        // TODO: remove this magic angle
        // double angleError = MathUtil.angleModulus(Math.PI / 2 - angle);
        return positionError.getMagnitude() < 2;
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.nextCommand(m_robot, this);
    }

    private void goToGoal() {
        Pose2d pose = m_robot.getPose();
        Vector2 position = new Vector2(pose.getX(), pose.getY());
        Vector2 positionError = position.to(m_robot.sourcePosition());
        m_robot.getRobotBody().applyForce(positionError.setMagnitude(500));
    }

}

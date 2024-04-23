package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.robot.RobotSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreAmp extends Command {
    private final Alliance m_alliance;
    private final RobotSubsystem m_robot;
    private final Tactics m_tactics;

    public ScoreAmp(Alliance alliance, RobotSubsystem robot) {
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
        Vector2 positionError = position.to(m_robot.ampPosition());
        double angle = pose.getRotation().getRadians();
        // TODO: remove this magic angle
        double angleError = MathUtil.angleModulus(Math.PI / 2 - angle);
        return (Math.abs(positionError.x) < 0.1
                && Math.abs(positionError.y) < 0.05
                && Math.abs(angleError) < 0.05);
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    private void goToGoal() {
        Pose2d pose = m_robot.getPose();
        Vector2 position = new Vector2(pose.getX(), pose.getY());
        Vector2 positionError = position.to(m_robot.ampPosition());
        double angle = pose.getRotation().getRadians();
        // TODO: remove this magic angle
        double angleError = MathUtil.angleModulus(Math.PI / 2 - angle);
        m_robot.getRobotBody().applyForce(positionError.setMagnitude(200));
        m_robot.getRobotBody().applyTorque(angleError * 50);
    }
}

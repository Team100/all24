package org.team100.commands;

import org.team100.robot.RobotSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private final RobotSubsystem m_robot;

    public NonplayerDefault(RobotSubsystem robot) {
        m_robot = robot;
        addRequirements(robot);
    }

    @Override
    public void execute() {
        m_robot.getRobotBody().avoidObstacles();
        avoidEdges();
    }

    /**
     * Avoid the edges of the field
     */
    private void avoidEdges() {
        Pose2d pose = m_robot.getPose();
        if (pose.getX() < 1)
            m_robot.apply(100, 0, 0);
        if (pose.getX() > 15)
            m_robot.apply(-100, 0, 0);
        if (pose.getY() < 1)
            m_robot.apply(0, 100, 0);
        if (pose.getY() > 7)
            m_robot.apply(0, -100, 0);
    }
}

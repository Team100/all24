package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotSubsystem;
import org.team100.sim.Heuristics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private static final int kSteer = 500;

    private final RobotSubsystem m_robot;

    public NonplayerDefault(RobotSubsystem robot) {
        m_robot = robot;
        addRequirements(robot);
    }

    @Override
    public void execute() {
        // m_robot.getRobotBody().avoidObstacles();
        avoidObstacles();
        avoidEdges();
    }

    /**
     * Avoid the edges of the field.
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

    /**
     * Avoid fixed obstacles.
     */
    private void avoidObstacles() {
        Pose2d position = m_robot.getPose();
        FieldRelativeVelocity velocity = m_robot.getVelocity();
        for (Translation2d body : m_robot.getRobotBody().getWorld().getObstacles()) {

            double distance = position.getTranslation().getDistance(body);
            if (distance > 4) // ignore far-away obstacles
                continue;

            Vector2 steer = Heuristics.steerToAvoid(
                    new Vector2(position.getX(), position.getY()),
                    new Vector2(velocity.x(), velocity.y()),
                    new Vector2(body.getX(), body.getY()), 1);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kSteer);
            m_robot.apply(force.x, force.y, 0);
        }
    }
}

package org.team100.commands;

import java.util.Map;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.FieldMap;
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
        avoidObstacles();
        avoidEdges();
        avoidSubwoofers();
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
     * Avoid the subwoofers.
     */
    private void avoidSubwoofers() {
        Pose2d pose = m_robot.getPose();
        for (Map.Entry<String, Pose2d> entry : FieldMap.subwoofers.entrySet()) {
            Translation2d target = entry.getValue().getTranslation();
            Translation2d robotRelativeToTarget = pose.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < 3) {
                // force goes as 1/r.
                Translation2d force = robotRelativeToTarget.times(500 / (norm * norm));
                m_robot.apply(force.getX(), force.getY(), 0);
            }
        }
    }

    /**
     * Avoid the stage posts.
     */
    private void avoidObstacles() {
        Pose2d myPosition = m_robot.getPose();
        FieldRelativeVelocity velocity = m_robot.getVelocity();
        for (Pose2d pose : FieldMap.stagePosts.values()) {
            Translation2d obstacleLocation = pose.getTranslation();
            double distance = myPosition.getTranslation().getDistance(obstacleLocation);
            if (distance > 4) // ignore far-away obstacles
                continue;
            Vector2 steer = Heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(velocity.x(), velocity.y()),
                    new Vector2(obstacleLocation.getX(), obstacleLocation.getY()), 1);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kSteer);
            m_robot.apply(force.x, force.y, 0);
        }
    }
}

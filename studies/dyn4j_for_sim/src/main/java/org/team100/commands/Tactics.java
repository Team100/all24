package org.team100.commands;

import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.planner.AvoidEdges;
import org.team100.planner.AvoidSubwoofers;
import org.team100.planner.ObstacleRepulsion;
import org.team100.planner.RobotRepulsion;
import org.team100.planner.SteerAroundObstacles;
import org.team100.planner.SteerAroundRobots;
import org.team100.planner.Tactic;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;

/**
 * Low level drive motion heuristics that can be used by any command.
 * 
 * Pointwise repulsive forces are inversely proportional to distance, like
 * gravity or electrostatics in two dimensions.
 */
public class Tactics {
    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final Tactic m_avoidEdges;
    private final Tactic m_avoidSubwoofers;
    private final Tactic m_steerAroundObstacles;
    private final Tactic m_obstacleRepulsion;
    private final Tactic m_steerAroundRobots;
    private final Tactic m_robotRepulsion;

    public Tactics(DriveSubsystem drive, CameraSubsystem camera) {
        m_drive = drive;
        m_camera = camera;
        m_avoidEdges = new AvoidEdges(m_drive);
        m_avoidSubwoofers = new AvoidSubwoofers(m_drive);
        m_steerAroundObstacles = new SteerAroundObstacles(m_drive);
        m_obstacleRepulsion = new ObstacleRepulsion(m_drive);
        m_steerAroundRobots = new SteerAroundRobots(m_drive, m_camera);
        m_robotRepulsion = new RobotRepulsion(m_drive, m_camera);
    }

    /**
     * Output is clamped to feasible v and omega.
     * 
     * @param avoidObstacles defenders don't care about obstacles
     * @param avoidEdges     some goals are near the edge, so turn this off.
     * @param avoidRobots    defenders don't care about robots
     */
    public FieldRelativeVelocity apply(
            FieldRelativeVelocity desired,
            boolean avoidObstacles,
            boolean avoidEdges,
            boolean avoidRobots,
            boolean debug) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        if (avoidObstacles) {
            v = v.plus(m_steerAroundObstacles.apply(desired, debug));
            v = v.plus(m_obstacleRepulsion.apply(desired, debug));
        }
        if (avoidEdges)
            v = v.plus(m_avoidEdges.apply(desired, debug));
        v = v.plus(m_avoidSubwoofers.apply(desired, debug));
        if (avoidRobots) {
            v = v.plus(m_steerAroundRobots.apply(desired, debug));
            v = v.plus(m_robotRepulsion.apply(desired, debug));
        }
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        return v;
    }

}

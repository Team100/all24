package org.team100.lib.planner;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Debug;

import edu.wpi.first.math.geometry.Translation2d;

/** Steering heuristics to avoid fixed and moving obstacles. */
public class Heuristics {

    private final boolean m_debug;

    public Heuristics(boolean debug) {
        m_debug = debug && Debug.enable();
    }

    /**
     * Return a velocity vector that, _if added to the supplied velocity,_ will
     * yield
     * the minimum distance when we pass by.
     *
     * If the target is behind us, return zero force.
     */
    public FieldRelativeVelocity steerToAvoid(
            Translation2d position,
            FieldRelativeVelocity velocity,
            Translation2d targetPosition,
            double distance) {
        Translation2d closestApproachPoint = closestApproach(
                position, velocity, targetPosition);
        if (closestApproachPoint == null) {
            return FieldRelativeVelocity.zero();
        }
        double distanceToGo = closestApproachPoint.minus(position).getNorm();
        if (distanceToGo < 1e-3) {
            return FieldRelativeVelocity.zero();
        }
        double timeToGo = distanceToGo / velocity.norm();

        if (m_debug)
            System.out.printf(" timeToGo %5.3f", timeToGo);

        Translation2d closestApproachRelative = closestApproachPoint.minus(targetPosition);

        double closestApproachDistance = closestApproachRelative.getNorm();
        // if the closest approach distance is zero, try to stop.
        if (closestApproachDistance < 1e-3) {
            return velocity.times(-1.0);
        }
        // this is the extra cross-track distance we need
        double steer = Math.max(0, distance - closestApproachDistance);
        double steerVelocity = steer / timeToGo;
        if (m_debug)
            System.out.printf(" steerVelocity %5.3f", steerVelocity);
        // TODO: i think this logic is wrong.
        Translation2d foo = closestApproachRelative.times(steerVelocity / closestApproachDistance);
        return new FieldRelativeVelocity(foo.getX(), foo.getY(), 0);
    }

    /**
     * Given our current location/velocity and a fixed target, return the point of
     * closest approach.
     * 
     * If our velocity is zero, then return null.
     * 
     * If the target is behind us, then return null.
     * 
     * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#
     * Vector_formulation
     * 
     * @param position       robot position, field-relative
     * @param velocity       robot velocity, field-relative
     * @param targetPosition
     */
    public Translation2d closestApproach(
            Translation2d position,
            FieldRelativeVelocity velocity,
            Translation2d targetPosition) {
        if (velocity.norm() < 1e-3) {
            // motionless
            return null;
        }

        FieldRelativeVelocity n = velocity.normalize();
        Translation2d targetRelative = targetPosition.minus(position);
        double magnitude = GeometryUtil.dot(targetRelative, n);
        if (magnitude < 0) {
            // target is behind us
            return null;
        }
        FieldRelativeVelocity projection = n.times(magnitude);
        Translation2d translation = new Translation2d(projection.x(), projection.y());
        return position.plus(translation);
    }

    /**
     * Given our current location/velocity and the location/velocity of a target,
     * return the point of closest approach.
     * 
     * This is the same as the zero-velocity version, but in the target's reference
     * frame instead of the field frame.
     * 
     * @param position       of the robot, field-relative
     * @param velocity       of the robot, field-relative
     * @param targetPosition field-relative
     * @param targetVelocity field-relative
     */
    public Translation2d closestApproach(
            Translation2d position,
            FieldRelativeVelocity velocity,
            Translation2d targetPosition,
            FieldRelativeVelocity targetVelocity) {
        FieldRelativeVelocity targetRelativeVelocity = velocity.minus(targetVelocity);
        Translation2d targetRelativeClosestApproach = closestApproach(
                position,
                targetRelativeVelocity,
                targetPosition);
        if (targetRelativeClosestApproach == null) {
            return null;
        }
        // TODO: this is wrong, need to know the time
        Translation2d translation = new Translation2d(targetVelocity.x(), targetVelocity.y());
        return targetRelativeClosestApproach.plus(translation);
    }
}

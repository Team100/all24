package org.team100.planner;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.util.Debug;

public class Heuristics {

    private final boolean m_debug;

    public Heuristics(boolean debug) {
        m_debug = debug && Debug.enable();
    }

    /**
     * Return a velocity vector that, if added to the supplied velocity, will yield
     * the minimum distance when we pass by.
     *
     * If the target is behind us, return zero force.
     * TODO: use wpilib types
     */
    public Vector2 steerToAvoid(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition,
            double distance) {
        Vector2 closestApproachPoint = closestApproach(
                position, velocity, targetPosition);
        if (closestApproachPoint == null) {
            return new Vector2();
        }
        double distanceToGo = closestApproachPoint.difference(position).getMagnitude();
        if (distanceToGo < 1e-3) {
            return new Vector2();
        }
        double timeToGo = distanceToGo / velocity.getMagnitude();

        if (m_debug)
            System.out.printf(" timeToGo %5.3f", timeToGo);

        Vector2 closestApproachRelative = closestApproachPoint.difference(targetPosition);

        double closestApproachDistance = closestApproachRelative.getMagnitude();
        // if the closest approach distance is zero, try to stop.
        if (closestApproachDistance < 1e-3) {
            return velocity.getNegative();
        }
        // this is the extra cross-track distance we need
        double steer = Math.max(0, distance - closestApproachDistance);
        double steerVelocity = steer / timeToGo;
        if (m_debug)
            System.out.printf(" steerVelocity %5.3f", steerVelocity);
        return closestApproachRelative.setMagnitude(steerVelocity);
    }

    /*
     * Given our current location/velocity and a fixed target, return the point of
     * closest approach.
     * 
     * If our velocity is zero, then return null.
     * 
     * If the target is behind us, then return null.
     * 
     * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#
     * Vector_formulation
     */
    public Vector2 closestApproach(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition) {
        if (velocity.getMagnitude() < 1e-3) {
            // motionless
            return null;
        }

        Vector2 n = velocity.getNormalized();
        Vector2 targetRelative = targetPosition.difference(position);
        double magnitude = targetRelative.dot(n);
        if (magnitude < 0) {
            // target is behind us
            return null;
        }
        Vector2 projection = n.product(magnitude);
        return position.sum(projection);
    }

    /**
     * Given our current location/velocity and the location/velocity of a target,
     * return the point of closest approach.
     * 
     * This is the same as the zero-velocity version, but in the target's reference
     * frame instead of the field frame.
     */
    public Vector2 closestApproach(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition,
            Vector2 targetVelocity) {
        Vector2 targetRelativeVelocity = velocity.difference(targetVelocity);
        Vector2 targetRelativeClosestApproach = closestApproach(
                position,
                targetRelativeVelocity,
                targetPosition);
        if (targetRelativeClosestApproach == null) {
            return null;
        }
        // TODO: this is wrong, need to know the time
        return targetRelativeClosestApproach.add(targetVelocity);
    }
}

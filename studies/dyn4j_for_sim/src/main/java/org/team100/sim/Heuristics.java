package org.team100.sim;

import org.dyn4j.geometry.Vector2;

public class Heuristics {

    /**
     * Return a force vector.
     * TODO: use wpilib types
     */
    public static Vector2 steerToAvoid(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition,
            double distance) {
        Vector2 closestApproachPoint = closestApproach(
                position, velocity, targetPosition);
        Vector2 closestApproachRelative = closestApproachPoint.difference(targetPosition);
        double closestApproachDistance = closestApproachRelative.getMagnitude();
        // if the closest approach distance is zero, try to stop.
        if (closestApproachDistance < 1e-3) {
            return velocity.getNegative();
        }
        double steer = Math.max(0, distance - closestApproachDistance);
        return closestApproachRelative.setMagnitude(steer);
    }

    /*
     * Given our current location/velocity and a fixed target, return the point of
     * closest approach.
     * 
     * If our velocity is zero, then the closest approach is our current position,
     * since it never changes.
     * 
     * If the target is behind us, then the closest approach is our current
     * position.
     * 
     * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#
     * Vector_formulation
     */
    public static Vector2 closestApproach(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition) {
        if (velocity.getMagnitude() < 1e-3) {
            // motionless
            return position;
        }

        Vector2 n = velocity.getNormalized();
        Vector2 targetRelative = targetPosition.difference(position);
        double magnitude = targetRelative.dot(n);
        if (magnitude < 0) {
            // target is behind us
            return position;
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
    public static Vector2 closestApproach(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition,
            Vector2 targetVelocity) {
        Vector2 targetRelativeVelocity = velocity.difference(targetVelocity);
        Vector2 targetRelativeClosestApproach = closestApproach(
                position,
                targetRelativeVelocity,
                targetPosition);
        // TODO: this is wrong, need to know the time
        return targetRelativeClosestApproach.add(targetVelocity);
    }

    private Heuristics() {
        //
    }
}

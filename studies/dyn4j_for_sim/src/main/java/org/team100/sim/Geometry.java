package org.team100.sim;

import org.dyn4j.geometry.Vector2;


public class Geometry {

    /** Return a force vector. */
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
        Vector2 projection = n.product(targetRelative.dot(n));
        return position.sum(projection);
    }

    /**
     * Given our current location/velocity and the location/velocity of a target,
     * return the point of closest approach.
     * 
     * https://en.wikipedia.org/wiki/Distance_of_closest_approach
     */
    public static void closestApproach(
            Vector2 position,
            Vector2 velocity,
            Vector2 targetPosition,
            Vector2 targetVelocity) {

    }

}

package org.team100.controllib.reference;

/**
 * ReferenceGenerator provides feasible trajectories of full state references
 * for control, based on full, partial, and/or infeasible state input, and a
 * supplied estimator.
 * 
 * For path planning, supply the endpoint and constraints. A timed trajectory
 * will be created that satisfies the input. Updating the goal or constraints
 * will regenerate the full trajectory.
 * 
 * The trajectory and its first derivative (which is used directly by
 * feedforward) can be sampled for any point in time.
 * 
 * Manual full-state control works the sme way, e.g. for rotation snaps.
 * 
 * Manual partial-state control, e.g. supplying velocity, uses the estimator to
 * supplement the input.
 * 
 * This is inspired by 254's SetpointGenerator and SwerveSetpointGenerator.
 */
public class ReferenceGenerator {
    public static class Constraint {
        public double jerk;
        public double accel;
        public double vel;
    }

    public static class State {
        public double position;
        public double velocity;
    }

    public static class Profile {

    }



    public ReferenceGenerator() {

    }

    /**
     * @return the reference for the specified time
     */
    public void getR(double tSec) {

    }

    /**
     * The derivative of the trajectory is used directly by feedforward.
     * 
     * @return the first derivative of the trajectory at the specified time
     */
    public void getRDot(double tSec) {

    }

}

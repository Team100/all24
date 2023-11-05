package org.team100.plotter;

import edu.wpi.first.math.trajectory.Trajectory;

/**
 * Represents a plotter operation.
 * 
 * The plotter can really just do one thing: follow a trajectory, with the pen
 * up or down.
 */
public class Operation {
    private final boolean penDown;
    private final Trajectory trajectory;

    public Operation(boolean penDown, Trajectory trajectory) {
        this.penDown = penDown;
        this.trajectory = trajectory;
    }

    public boolean isPenDown() {
        return penDown;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}

package org.team100.lib.path;

/**
 * Allows sampling a path by the index of points that make up a path.
 */
public class PathIndexSampler {
    private final Path trajectory;

    public PathIndexSampler(Path trajectory) {
        this.trajectory = trajectory;
    }

    public PathSamplePoint sample(double index) {
        return trajectory.getInterpolated(index);
    }

    public double last_interpolant() {
        return Math.max(0.0, trajectory.length() - 1);
    }

    public double first_interpolant() {
        return 0.0;
    }
}
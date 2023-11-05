package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.timing.TimedPose;

/**
 * Represents a 2d path with heading and a schedule.
 * 
 * As of 2023 a trajectory is not two things (path and heading) it's one thing, each path point includes heading.
 */
public class Trajectory {
    protected final List<TrajectoryPoint> points_;

    public Trajectory() {
        points_ = new ArrayList<>();
    }

    public Trajectory(final List<TimedPose> states) {
        points_ = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            points_.add(new TrajectoryPoint(states.get(i), i));
        }
    }

    public boolean isEmpty() {
        return points_.isEmpty();
    }

    public int length() {
        return points_.size();
    }

    public TrajectoryPoint getLastPoint() {
        return points_.get(length() - 1);
    }

    public TrajectoryPoint getPoint(final int index) {
        return points_.get(index);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": state: ");
            builder.append(getPoint(i).state());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }
}

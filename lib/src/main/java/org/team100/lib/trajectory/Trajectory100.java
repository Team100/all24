package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.timing.TimedPose;

/**
 * Represents a 2d path with heading and a schedule.
 * 
 * As of 2023 a trajectory is not two things (path and heading) it's one thing, each path point includes heading.
 */
public class Trajectory100 {
    protected final List<TrajectoryPoint> m_points;

    public Trajectory100() {
        m_points = new ArrayList<>();
    }

    public Trajectory100(final List<TimedPose> states) {
        m_points = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            m_points.add(new TrajectoryPoint(states.get(i), i));
        }
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public TrajectoryPoint getLastPoint() {
        return m_points.get(length() - 1);
    }

    public TrajectoryPoint getPoint(final int index) {
        return m_points.get(index);
    }

    public List<TrajectoryPoint> getPoints() {
        return m_points;
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

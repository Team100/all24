package org.team100.lib.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Represents a 2d path with heading.
 * 
 * There's no timing information here. For that, see Trajectory.
 */
public class Path100 {
    private final List<PathPoint> m_points;

    public Path100() {
        m_points = new ArrayList<>();
    }

    public Path100(final List<Pose2dWithMotion> states) {
        m_points = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            m_points.add(new PathPoint(states.get(i), i));
        }
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public PathPoint getPoint(final int index) {
        return m_points.get(index);
    }

    public PathSamplePoint getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) {
            PathPoint point = getPoint(0);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        } else if (index >= length() - 1) {
            PathPoint point = getPoint(length() - 1);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        }
        final int i = (int) Math.floor(index);
        final double frac = index - i;
        if (frac <= Double.MIN_VALUE) {
            PathPoint point = getPoint(i);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            PathPoint point = getPoint(i + 1);
            return new PathSamplePoint(point.state(), point.index(), point.index());
        } else {
            return new PathSamplePoint(
                    getPoint(i).state().interpolate(getPoint(i + 1).state(), frac),
                    i,
                    i + 1);
        }
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i).state());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }
}

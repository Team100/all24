package org.team100.lib.planner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ForceViz {
    private static final double kScale = 0.5;

    private final List<Double> m_tactics = new ArrayList<>();
    private final List<Double> m_desired = new ArrayList<>();
    private final DoubleArrayLogger m_log_tactics;
    private final DoubleArrayLogger m_log_desired;

    public ForceViz(LoggerFactory fieldLogger) {
        m_log_tactics = fieldLogger.doubleArrayLogger(Level.TRACE, "tactics");
        m_log_desired = fieldLogger.doubleArrayLogger(Level.TRACE, "desired");
    }

    public void tactics(Translation2d p, FieldRelativeVelocity v) {
        put(m_tactics, p, v);
    }

    public void desired(Translation2d p, FieldRelativeVelocity v) {
        put(m_desired, p, v);
    }

    private void put(List<Double> f, Translation2d p, FieldRelativeVelocity v) {
        // ignore small forces
        if (v.norm() < 0.1)
            return;
        Optional<Rotation2d> angle = v.angle();
        if (angle.isEmpty())
            return;
        double direction = angle.get().getDegrees();
        double x = p.getX() - v.x() * kScale;
        double y = p.getY() - v.y() * kScale;
        f.add(x);
        f.add(y);
        f.add(direction);
    }

    public void render() {
        m_log_tactics.log(() -> m_tactics.stream().mapToDouble(Double::doubleValue).toArray());
        m_log_desired.log(() -> m_desired.stream().mapToDouble(Double::doubleValue).toArray());
    }

}
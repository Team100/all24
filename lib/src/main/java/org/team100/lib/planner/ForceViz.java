package org.team100.lib.planner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;

import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ForceViz {
    // see simgui.json for these names
    private static final String kTactics = "tactics";
    private static final String kDesired = "desired";
    private static final double kScale = 0.5;

    private final SupplierLogger m_logger = Telemetry.get().fieldLogger(false, false);
    private final Map<String, List<Double>> items;

    public ForceViz() {
        items = new HashMap<>();
    }

    public void tactics(Translation2d p, FieldRelativeVelocity v) {
        put(kTactics, p, v);
    }

    public void desired(Translation2d p, FieldRelativeVelocity v) {
        put(kDesired, p, v);
    }

    private void put(String name, Translation2d p, FieldRelativeVelocity v) {
        // ignore small forces
        if (v.norm() < 0.1)
            return;
        Optional<Rotation2d> angle = v.angle();
        if (angle.isEmpty())
            return;
        items.putIfAbsent(name, new ArrayList<>());
        double direction = angle.get().getDegrees();
        double x = p.getX() - v.x() * kScale;
        double y = p.getY() - v.y() * kScale;
        List<Double> f = items.get(name);
        f.add(x);
        f.add(y);
        f.add(direction);
    }

    public void render() {
        for (Entry<String, List<Double>> entry : items.entrySet()) {
            m_logger.logDoubleObjArray(
                    Level.DEBUG,
                    entry.getKey(),
                    () -> entry.getValue().toArray(new Double[0]));
            entry.getValue().clear();
        }
    }

}
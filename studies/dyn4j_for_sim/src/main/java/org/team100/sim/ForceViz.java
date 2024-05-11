package org.team100.sim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;

public class ForceViz {
    private static final Telemetry t = Telemetry.get();
    private static final String kField = "field";
    private static final double kScale = 0.5;

    public static final Map<String, List<Double>> items = new HashMap<>();

    public static void put(String name, Pose2d p, FieldRelativeVelocity v) {
        // ignore small forces
        if (v.norm() < 0.1)
            return;
        items.putIfAbsent(name, new ArrayList<>());
        double direction = v.angle().getDegrees();
        double x = p.getX() - v.x() * kScale;
        double y = p.getY() - v.y() * kScale;
        List<Double> f = items.get(name);
        f.add(x);
        f.add(y);
        f.add(direction);
    }

    public static void render() {
        for (Entry<String, List<Double>> entry : items.entrySet()) {
            t.log(
                    Level.DEBUG,
                    kField,
                    entry.getKey(),
                    entry.getValue().toArray(new Double[0]));
            entry.getValue().clear();
        }
    }

    private ForceViz() {
        // TODO: make this not a static thing
    }
}

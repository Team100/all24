package org.team100.lib.telemetry;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.team100.lib.controller.State100;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Chronos.Sample;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public class NTLogger implements Logger {
    private final Telemetry m_telemetry;
    private final String m_root;
    private final BooleanSupplier m_enabled;

    NTLogger(Telemetry telemetry, String root, BooleanSupplier enabled) {
        m_telemetry = telemetry;
        m_root = root;
        m_enabled = enabled;
    }

    /** Adds a slash between the root and the stem */
    @Override
    public Logger child(String stem) {
        return new NTLogger(m_telemetry, m_root + "/" + stem, m_enabled);
    }

    @Override
    public boolean enabled() {
        return m_enabled.getAsBoolean();
    }

    @Override
    public void logBoolean(Level level, String leaf, boolean val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        String key = append(m_root, leaf);
        print(key, val);
        pub(key, k -> {
            BooleanTopic t = m_telemetry.inst.getBooleanTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, BooleanPublisher.class).set(val);
    }

    /**
     * This is for tuning through glass.
     * Remember that the values don't survive restarts, so
     * you should write them down.
     */
    @Override
    public void register(Level level, String leaf, double initial, DoubleConsumer consumer) {
        if (!m_telemetry.m_level.admit(level))
            return;
        String k = append(m_root, leaf);
        DoubleTopic t = m_telemetry.inst.getDoubleTopic(k);
        t.publish().set(initial);
        t.setRetained(true);
        m_telemetry.inst.addListener(t, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> consumer.accept(e.valueData.value.getDouble()));
    }

    // using a supplier here is faster in the non-logging case.
    @Override
    public void logDouble(Level level, String leaf, DoubleSupplier vals) {
        Sample s = m_telemetry.m_chronos.sample(Telemetry.kName);
        try {
            if (!enabled())
                return;
            if (!m_telemetry.m_level.admit(level))
                return;
            double val = vals.getAsDouble();
            String key = append(m_root, leaf);
            print(key, val);
            pub(key, k -> {
                DoubleTopic t = m_telemetry.inst.getDoubleTopic(k);
                t.publish();
                t.setRetained(true);
                return t.publish();
            }, DoublePublisher.class).set(val);
        } finally {
            s.end();
        }
    }

    @Override
    public void log(Level level, String leaf, OptionalDouble val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        if (val.isEmpty()) {
            return;
        }
        logDouble(level, leaf, val::getAsDouble);
    }

    @Override
    public void log(Level level, String leaf, float val) {
        // if(val == null)
        // return;
        if (!m_telemetry.m_level.admit(level))
            return;
        String key = append(m_root, leaf);
        print(key, val);
        pub(key, k -> {
            DoubleTopic t = m_telemetry.inst.getDoubleTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, DoublePublisher.class).set(val);
    }

    @Override
    public void log(Level level, String leaf, double[] val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        String key = append(m_root, leaf);
        print(key, val);
        pub(key, k -> {
            DoubleArrayTopic t = m_telemetry.inst.getDoubleArrayTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, DoubleArrayPublisher.class).set(val);
    }

    @Override
    public void log(Level level, String leaf, Double[] val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        log(level, leaf, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public void log(Level level, String leaf, long val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        String key = append(m_root, leaf);
        print(key, val);
        pub(key, k -> {
            IntegerTopic t = m_telemetry.inst.getIntegerTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, IntegerPublisher.class).set(val);
    }

    @Override
    public void log(Level level, String leaf, String val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        String key = append(m_root, leaf);
        print(key, val);
        pub(key, k -> {
            StringTopic t = m_telemetry.inst.getStringTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, StringPublisher.class).set(val);
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    @Override
    public void log(Level level, String leaf, Supplier<String[]> val) {
        if (!m_telemetry.m_level.admit(level))
            return;
        String key = append(m_root, leaf);
        print(key, val);
        pub(key, k -> {
            StringArrayTopic t = m_telemetry.inst.getStringArrayTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, StringArrayPublisher.class).set(val.get());
    }

    @Override
    public void log(Level level, String leaf, Enum<?> val) {
        log(level, leaf, val.name());
    }

    @Override
    public void log(Level level, String leaf, Pose2d val) {
        log(level, append(leaf, "translation"), val.getTranslation());
        log(level, append(leaf, "rotation"), val.getRotation());
    }

    @Override
    public void log(Level level, String leaf, Translation2d val) {
        logDouble(level, append(leaf, "x"), val::getX);
        logDouble(level, append(leaf, "y"), val::getY);
    }

    @Override
    public void log(Level level, String leaf, Vector2d val) {
        logDouble(level, append(leaf, "x"), val::getX);
        logDouble(level, append(leaf, "y"), val::getY);

    }

    @Override
    public void log(Level level, String leaf, Rotation2d val) {
        logDouble(level, append(leaf, "rad"), val::getRadians);
    }

    @Override
    public void log(Level level, String leaf, TrajectorySamplePoint val) {
        log(level, append(leaf, "state"), val.state());
    }

    @Override
    public void log(Level level, String leaf, TimedPose val) {
        log(level, append(leaf, "posestate"), val.state());
        logDouble(level, append(leaf, "time"), val::getTimeS);
        logDouble(level, append(leaf, "velocity"), val::velocityM_S);
        logDouble(level, append(leaf, "accel"), val::acceleration);
    }

    @Override
    public void log(Level level, String leaf, PoseWithCurvature val) {
        log(level, append(leaf, "pose"), val.poseMeters);
    }

    @Override
    public void log(Level level, String leaf, Pose2dWithMotion val) {
        log(level, append(leaf, "pose"), val.getPose());
        Optional<Rotation2d> course = val.getCourse();
        if (course.isPresent()) {
            log(level, append(leaf, "course"), course.get());
        }
    }

    @Override
    public void log(Level level, String leaf, Twist2d val) {
        logDouble(level, append(leaf, "dx"), () -> val.dx);
        logDouble(level, append(leaf, "dy"), () -> val.dy);
        logDouble(level, append(leaf, "dtheta"), () -> val.dtheta);
    }

    @Override
    public void log(Level level, String leaf, ChassisSpeeds val) {
        logDouble(level, append(leaf, "vx m_s"), () -> val.vxMetersPerSecond);
        logDouble(level, append(leaf, "vy m_s"), () -> val.vyMetersPerSecond);
        logDouble(level, append(leaf, "omega rad_s"), () -> val.omegaRadiansPerSecond);
    }

    @Override
    public void log(Level level, String leaf, FieldRelativeVelocity val) {
        logDouble(level, append(leaf, "x m_s"), val::x);
        logDouble(level, append(leaf, "y m_s"), val::y);
        logDouble(level, append(leaf, "theta rad_s"), val::theta);
    }

    @Override
    public void log(Level level, String leaf, FieldRelativeAcceleration val) {
        logDouble(level, append(leaf, "x m_s_s"), val::x);
        logDouble(level, append(leaf, "y m_s_s"), val::y);
        logDouble(level, append(leaf, "theta rad_s_s"), val::theta);
    }

    @Override
    public void log(Level level, String leaf, State100 state) {
        logDouble(level, append(leaf, "x"), state::x);
        logDouble(level, append(leaf, "v"), state::v);
        logDouble(level, append(leaf, "a"), state::a);
    }

    @Override
    public void log(Level level, String leaf, SwerveState state) {
        log(level, append(leaf, "x"), state.x());
        log(level, append(leaf, "y"), state.y());
        log(level, append(leaf, "theta"), state.theta());
    }

    @Override
    public void log(Level level, String leaf, SwerveModulePosition val) {
        logDouble(level, append(leaf, "distance"), () -> val.distanceMeters);
        log(level, append(leaf, "angle"), val.angle);
    }

    @Override
    public void log(Level level, String leaf, ArmAngles angles) {
        logDouble(level, append(leaf, "th1"), () -> angles.th1);
        logDouble(level, append(leaf, "th2"), () -> angles.th2);
    }

    @Override
    public void log(Level level, String leaf, State state) {
        log(level, append(leaf, "pose"), state.poseMeters);
        logDouble(level, append(leaf, "curvature"), () -> state.curvatureRadPerMeter);
        logDouble(level, append(leaf, "velocity"), () -> state.velocityMetersPerSecond);
        logDouble(level, append(leaf, "accel"), () -> state.accelerationMetersPerSecondSq);
    }

    private <T extends Publisher> T pub(String key, Function<String, Publisher> fn, Class<T> pubClass) {
        Publisher publisher = m_telemetry.pubs.computeIfAbsent(valid(key), fn);
        if (!pubClass.isInstance(publisher))
            throw new IllegalArgumentException(
                    String.format("value type clash for key %s old %s new %s",
                            key,
                            publisher.getClass().getName(),
                            pubClass.getName()));
        return pubClass.cast(publisher);
    }

    private String valid(String key) {
        if (key.length() == 0)
            throw new IllegalArgumentException("empty key");
        if (key.charAt(0) != '/')
            throw new IllegalArgumentException("key must start with slash; you provided: " + key);
        return key;
    }

    /** Make a key for the root level (with a leading slash). */
    private static String append(String root, String leaf) {
        if (root.startsWith("/"))
            return root + "/" + leaf;
        return "/" + root + "/" + leaf;
    }

    private static void print(String key, boolean val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %b\n", key, val);
    }

    private void print(String key, Double val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %5.5f\n", key, val);
    }

    private void print(String key, float val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %5.5f\n", key, val);
    }

    private void print(String key, double[] val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, Arrays.toString(val));
    }

    private void print(String key, long val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %d\n", key, val);
    }

    private void print(String key, String val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, val);
    }

    private void print(String key, Supplier<String[]> val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, Arrays.toString(val.get()));
    }
}
package org.team100.lib.telemetry;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.ConcurrentHashMap;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Simple logging wrapper.
 * 
 * Use keys of the form "/foo/bar"; the slashes separate levels in the tree.
 * 
 * Don't use a slash for any other reason, e.g. for "meters per second" don't
 * say "m/s" say "m_s"
 * 
 * Logged items are "retained" which means they persist even after the logging
 * stops; this means you should see the latest values after disabling the robot.
 */
public class Telemetry {
    public enum Level {
        /** Log nothing for maximum speed */
        SILENT(1),
        /** Serious problems */
        ERROR(2),
        /** Not-serious problems */
        WARN(3),
        /** useful for normal operations */
        INFO(4),
        /** primarily useful during development */
        DEBUG(5),
        /** slow, shows absolutely everything */
        TRACE(6);

        private int priority;

        private Level(int priority) {
            this.priority = priority;
        }

        public boolean admit(Level other) {
            return this.priority >= other.priority;
        }
    }

    /**
     * useful for troubleshooting unit tests. it's quite slow.
     */
    private static final boolean kAlsoPrint = false;
    private static final Telemetry instance = new Telemetry();
    private static final String kName = "Telemetry";

    private final Chronos m_chronos = Chronos.get();
    private final NetworkTableInstance inst;
    private final Map<String, Publisher> pubs;
    private Level m_level;

    /**
     * Uses the default network table instance.
     * Clients should use the static instance, not the constructor.
     */
    private Telemetry() {
        inst = NetworkTableInstance.getDefault();
        pubs = new ConcurrentHashMap<>();
        // this will be overridden by {@link TelemetryLevelPoller}
        m_level = Level.TRACE;
        DataLogManager.start();
    }

    void setLevel(Level level) {
        m_level = level;
    }

    public static Telemetry get() {
        return instance;
    }

    public void log(Level level, String root, String leaf, boolean val) {
        if (!m_level.admit(level))
            return;
        String key = Telemetry.append(root, leaf);
        print(key, val);
        pub(key, k -> {
            BooleanTopic t = inst.getBooleanTopic(k);
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
    public void register(Level level, String root, String leaf, double initial, DoubleConsumer consumer) {
        if (!m_level.admit(level))
            return;
        String k = Telemetry.append(root, leaf);
        DoubleTopic t = inst.getDoubleTopic(k);
        t.publish().set(initial);
        t.setRetained(true);
        inst.addListener(t, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> consumer.accept(e.valueData.value.getDouble()));
    }

    public void log(Level level, String root, String leaf, Double val) {
        Sample s = m_chronos.sample(kName);
        try {
            if (val == null)
                return;
            if (!m_level.admit(level))
                return;
            String key = Telemetry.append(root, leaf);
            print(key, val);
            pub(key, k -> {
                DoubleTopic t = inst.getDoubleTopic(k);
                t.publish();
                t.setRetained(true);
                return t.publish();
            }, DoublePublisher.class).set(val);
        } finally {
            s.end();
        }
    }

    public void log(Level level, String root, String leaf, OptionalDouble val) {
        if (!m_level.admit(level))
            return;
        if (val.isEmpty()) {
            return;
        }
        log(level, root, leaf, val.getAsDouble());
    }

    // using a supplier here is faster in the non-logging case.
    public void log(Level level, String root, String leaf, DoubleSupplier val) {
        if (!m_level.admit(level))
            return;
        log(level, root, leaf, val.getAsDouble());
    }

    public void log(Level level, String root, String leaf, float val) {
        // if(val == null)
        // return;
        if (!m_level.admit(level))
            return;
        String key = Telemetry.append(root, leaf);
        print(key, val);
        pub(key, k -> {
            DoubleTopic t = inst.getDoubleTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, DoublePublisher.class).set(val);
    }

    public void log(Level level, String root, String leaf, double[] val) {
        if (!m_level.admit(level))
            return;
        String key = Telemetry.append(root, leaf);
        print(key, val);
        pub(key, k -> {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, DoubleArrayPublisher.class).set(val);
    }

    public void log(Level level, String root, String leaf, Double[] val) {
        if (!m_level.admit(level))
            return;
        log(level, root, leaf, Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    public void log(Level level, String root, String leaf, long val) {
        if (!m_level.admit(level))
            return;
        String key = Telemetry.append(root, leaf);
        print(key, val);
        pub(key, k -> {
            IntegerTopic t = inst.getIntegerTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, IntegerPublisher.class).set(val);
    }

    public void log(Level level, String root, String leaf, String val) {
        if (!m_level.admit(level))
            return;
        String key = Telemetry.append(root, leaf);
        print(key, val);
        pub(key, k -> {
            StringTopic t = inst.getStringTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, StringPublisher.class).set(val);
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    public void log(Level level, String root, String leaf, Supplier<String[]> val) {
        if (!m_level.admit(level))
            return;
        String key = Telemetry.append(root, leaf);
        print(key, val);
        pub(key, k -> {
            StringArrayTopic t = inst.getStringArrayTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, StringArrayPublisher.class).set(val.get());
    }

    public void log(Level level, String root, String leaf, Enum<?> val) {
        log(level, root, leaf, val.name());
    }

    public void log(Level level, String root, String leaf, Pose2d val) {
        log(level, Telemetry.append(root, leaf), "translation", val.getTranslation());
        log(level, Telemetry.append(root, leaf), "rotation", val.getRotation());
    }

    public void log(Level level, String root, String leaf, Translation2d val) {
        log(level, Telemetry.append(root, leaf), "x", val.getX());
        log(level, Telemetry.append(root, leaf), "y", val.getY());
    }

    public void log(Level level, String root, String leaf, Vector2d val) {
        log(level, Telemetry.append(root, leaf), "x", val.getX());
        log(level, Telemetry.append(root, leaf), "y", val.getY());

    }

    public void log(Level level, String root, String leaf, Rotation2d val) {
        log(level, Telemetry.append(root, leaf), "rad", val.getRadians());
    }

    public void log(Level level, String root, String leaf, TrajectorySamplePoint val) {
        log(level, Telemetry.append(root, leaf), "state", val.state());
    }

    public void log(Level level, String root, String leaf, TimedPose val) {
        log(level, Telemetry.append(root, leaf), "posestate", val.state());
        log(level, Telemetry.append(root, leaf), "time", val.getTimeS());
        log(level, Telemetry.append(root, leaf), "velocity", val.velocityM_S());
        log(level, Telemetry.append(root, leaf), "accel", val.acceleration());
    }

    public void log(Level level, String root, String leaf, PoseWithCurvature val) {
        log(level, Telemetry.append(root, leaf), "pose", val.poseMeters);
    }

    public void log(Level level, String root, String leaf, Pose2dWithMotion val) {
        log(level, Telemetry.append(root, leaf), "pose", val.getPose());
        Optional<Rotation2d> course = val.getCourse();
        if (course.isPresent()) {
            log(level, Telemetry.append(root, leaf), "course", course.get());
        }
    }

    public void log(Level level, String root, String leaf, Twist2d val) {
        log(level, Telemetry.append(root, leaf), "dx", val.dx);
        log(level, Telemetry.append(root, leaf), "dy", val.dy);
        log(level, Telemetry.append(root, leaf), "dtheta", val.dtheta);
    }

    public void log(Level level, String root, String leaf, ChassisSpeeds val) {
        log(level, Telemetry.append(root, leaf), "vx m_s", val.vxMetersPerSecond);
        log(level, Telemetry.append(root, leaf), "vy m_s", val.vyMetersPerSecond);
        log(level, Telemetry.append(root, leaf), "omega rad_s", val.omegaRadiansPerSecond);
    }

    public void log(Level level, String root, String leaf, FieldRelativeVelocity val) {
        log(level, Telemetry.append(root, leaf), "x m_s", val.x());
        log(level, Telemetry.append(root, leaf), "y m_s", val.y());
        log(level, Telemetry.append(root, leaf), "theta rad_s", val.theta());
    }

    public void log(Level level, String root, String leaf, FieldRelativeAcceleration val) {
        log(level, Telemetry.append(root, leaf), "x m_s_s", val.x());
        log(level, Telemetry.append(root, leaf), "y m_s_s", val.y());
        log(level, Telemetry.append(root, leaf), "theta rad_s_s", val.theta());
    }

    public void log(Level level, String root, String leaf, State100 state) {
        log(level, Telemetry.append(root, leaf), "x", state.x());
        log(level, Telemetry.append(root, leaf), "v", state.v());
        log(level, Telemetry.append(root, leaf), "a", state.a());
    }

    public void log(Level level, String root, String leaf, SwerveState state) {
        log(level, Telemetry.append(root, leaf), "x", state.x());
        log(level, Telemetry.append(root, leaf), "y", state.y());
        log(level, Telemetry.append(root, leaf), "theta", state.theta());
    }

    public void log(Level level, String root, String leaf, SwerveModulePosition val) {
        log(level, Telemetry.append(root, leaf), "distance", val.distanceMeters);
        log(level, Telemetry.append(root, leaf), "angle", val.angle);
    }

    public void log(Level level, String root, String leaf, ArmAngles angles) {
        log(level, Telemetry.append(root, leaf), "th1", angles.th1);
        log(level, Telemetry.append(root, leaf), "th2", angles.th2);
    }

    public void log(Level level, String root, String leaf, State state) {
        log(level, Telemetry.append(root, leaf), "pose", state.poseMeters);
        log(level, Telemetry.append(root, leaf), "curvature", state.curvatureRadPerMeter);
        log(level, Telemetry.append(root, leaf), "velocity", state.velocityMetersPerSecond);
        log(level, Telemetry.append(root, leaf), "accel", state.accelerationMetersPerSecondSq);
    }

    private <T extends Publisher> T pub(String key, Function<String, Publisher> fn, Class<T> pubClass) {
        Publisher publisher = pubs.computeIfAbsent(valid(key), fn);
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
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %b\n", key, val);
    }

    private void print(String key, Double val) {
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %5.5f\n", key, val);
    }

    private void print(String key, float val) {
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %5.5f\n", key, val);
    }

    private void print(String key, double[] val) {
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, Arrays.toString(val));
    }

    private void print(String key, long val) {
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %d\n", key, val);
    }

    private void print(String key, String val) {
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, val);
    }

    private void print(String key, Supplier<String[]> val) {
        if (!kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, Arrays.toString(val.get()));
    }
}
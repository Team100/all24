package org.team100.lib.telemetry;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.LongSupplier;
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
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.function.FloatSupplier;

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

    private boolean enabled() {
        return m_enabled.getAsBoolean();
    }

    private boolean allow(Level level) {
        if (m_telemetry.m_level == Level.COMP && level == Level.COMP) {
            // comp mode allows COMP level regardless of enablement.
            return true;
        }
        return enabled() && m_telemetry.m_level.admit(level);
    }

    @Override
    public void logBoolean(Level level, String leaf, BooleanSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        boolean val = vals.getAsBoolean();
        print(key, val);
        pub(key, k -> {
            BooleanTopic t = m_telemetry.inst.getBooleanTopic(k);
            BooleanPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, BooleanPublisher.class).set(val);
    }

    /**
     * This is for tuning through glass.
     * Remember that the values don't survive restarts, so
     * you should write them down.
     * TODO: get rid of this, we never use it.
     */
    @Override
    public void register(Level level, String leaf, double initial, DoubleConsumer consumer) {
        if (!allow(level))
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
            if (!allow(level))
                return;
            String key = append(m_root, leaf);
            double val = vals.getAsDouble();
            print(key, val);
            pub(key, k -> {
                DoubleTopic t = m_telemetry.inst.getDoubleTopic(k);
                DoublePublisher p = t.publish();
                t.setRetained(true);
                return p;
            }, DoublePublisher.class).set(val);
        } finally {
            s.end();
        }
    }

    @Override
    public void logOptionalDouble(Level level, String leaf, Supplier<OptionalDouble> vals) {
        if (!allow(level))
            return;
        OptionalDouble val = vals.get();
        if (val.isPresent()) {
            logDouble(level, leaf, val::getAsDouble);
        }
    }

    @Override
    public void logFloat(Level level, String leaf, FloatSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        float val = vals.getAsFloat();
        print(key, val);
        pub(key, k -> {
            FloatTopic t = m_telemetry.inst.getFloatTopic(k);
            FloatPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, FloatPublisher.class).set(val);
    }

    @Override
    public void logDoubleArray(Level level, String leaf, Supplier<double[]> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        double[] val = vals.get();
        print(key, val);
        pub(key, k -> {
            DoubleArrayTopic t = m_telemetry.inst.getDoubleArrayTopic(k);
            DoubleArrayPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, DoubleArrayPublisher.class).set(val);
    }

    @Override
    public void logDoubleObjArray(Level level, String leaf, Supplier<Double[]> vals) {
        if (!allow(level))
            return;
        Double[] val = vals.get();
        logDoubleArray(level, leaf, () -> Stream.of(val).mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public void logLong(Level level, String leaf, LongSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        long val = vals.getAsLong();
        print(key, val);
        pub(key, k -> {
            IntegerTopic t = m_telemetry.inst.getIntegerTopic(k);
            IntegerPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, IntegerPublisher.class).set(val);
    }

    @Override
    public void logString(Level level, String leaf, Supplier<String> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        String val = vals.get();
        print(key, val);
        pub(key, k -> {
            StringTopic t = m_telemetry.inst.getStringTopic(k);
            StringPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, StringPublisher.class).set(val);
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    @Override
    public void logStringArray(Level level, String leaf, Supplier<String[]> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        String[] val = vals.get();
        print(key, val);
        pub(key, k -> {
            StringArrayTopic t = m_telemetry.inst.getStringArrayTopic(k);
            StringArrayPublisher p = t.publish();
            t.setRetained(true);
            return p;
        }, StringArrayPublisher.class).set(val);
    }

    @Override
    public void logEnum(Level level, String leaf, Supplier<Enum<?>> val) {
        logString(level, leaf, () -> val.get().name());
    }

    @Override
    public void logPose2d(Level level, String leaf, Supplier<Pose2d> val) {
        logTranslation2d(level, append(leaf, "translation"), () -> val.get().getTranslation());
        logRotation2d(level, append(leaf, "rotation"), () -> val.get().getRotation());
    }

    @Override
    public void logTranslation2d(Level level, String leaf, Supplier<Translation2d> val) {
        logDouble(level, append(leaf, "x"), () -> val.get().getX());
        logDouble(level, append(leaf, "y"), () -> val.get().getY());
    }

    @Override
    public void logVector2d(Level level, String leaf, Supplier<Vector2d> val) {
        logDouble(level, append(leaf, "x"), () -> val.get().getX());
        logDouble(level, append(leaf, "y"), () -> val.get().getY());

    }

    @Override
    public void logRotation2d(Level level, String leaf, Supplier<Rotation2d> val) {
        logDouble(level, append(leaf, "rad"), () -> val.get().getRadians());
    }

    @Override
    public void logTrajectorySamplePoint(Level level, String leaf, Supplier<TrajectorySamplePoint> val) {
        logTimedPose(level, append(leaf, "state"), () -> val.get().state());
    }

    @Override
    public void logTimedPose(Level level, String leaf, Supplier<TimedPose> val) {
        logPose2dWithMotion(level, append(leaf, "posestate"), () -> val.get().state());
        logDouble(level, append(leaf, "time"), () -> val.get().getTimeS());
        logDouble(level, append(leaf, "velocity"), () -> val.get().velocityM_S());
        logDouble(level, append(leaf, "accel"), () -> val.get().acceleration());
    }

    @Override
    public void logPoseWithCurvature(Level level, String leaf, Supplier<PoseWithCurvature> val) {
        logPose2d(level, append(leaf, "pose"), () -> val.get().poseMeters);
    }

    @Override
    public void logPose2dWithMotion(Level level, String leaf, Supplier<Pose2dWithMotion> val) {
        logPose2d(level, append(leaf, "pose"), () -> val.get().getPose());
        Optional<Rotation2d> course = val.get().getCourse();
        if (course.isPresent()) {
            logRotation2d(level, append(leaf, "course"), course::get);
        }
    }

    @Override
    public void logTwist2d(Level level, String leaf, Supplier<Twist2d> val) {
        logDouble(level, append(leaf, "dx"), () -> val.get().dx);
        logDouble(level, append(leaf, "dy"), () -> val.get().dy);
        logDouble(level, append(leaf, "dtheta"), () -> val.get().dtheta);
    }

    @Override
    public void logChassisSpeeds(Level level, String leaf, Supplier<ChassisSpeeds> val) {
        logDouble(level, append(leaf, "vx m_s"), () -> val.get().vxMetersPerSecond);
        logDouble(level, append(leaf, "vy m_s"), () -> val.get().vyMetersPerSecond);
        logDouble(level, append(leaf, "omega rad_s"), () -> val.get().omegaRadiansPerSecond);
    }

    @Override
    public void logFieldRelativeVelocity(Level level, String leaf, Supplier<FieldRelativeVelocity> val) {
        logDouble(level, append(leaf, "x m_s"), () -> val.get().x());
        logDouble(level, append(leaf, "y m_s"), () -> val.get().y());
        logDouble(level, append(leaf, "theta rad_s"), () -> val.get().theta());
    }

    @Override
    public void logFieldRelativeAcceleration(Level level, String leaf, Supplier<FieldRelativeAcceleration> val) {
        logDouble(level, append(leaf, "x m_s_s"), () -> val.get().x());
        logDouble(level, append(leaf, "y m_s_s"), () -> val.get().y());
        logDouble(level, append(leaf, "theta rad_s_s"), () -> val.get().theta());
    }

    @Override
    public void logState100(Level level, String leaf, Supplier<State100> state) {
        logDouble(level, append(leaf, "x"), () -> state.get().x());
        logDouble(level, append(leaf, "v"), () -> state.get().v());
        logDouble(level, append(leaf, "a"), () -> state.get().a());
    }

    @Override
    public void logSwerveState(Level level, String leaf, Supplier<SwerveState> state) {
        logState100(level, append(leaf, "x"), () -> state.get().x());
        logState100(level, append(leaf, "y"), () -> state.get().y());
        logState100(level, append(leaf, "theta"), () -> state.get().theta());
    }

    @Override
    public void logSwerveModulePosition(Level level, String leaf, Supplier<SwerveModulePosition> val) {
        logDouble(level, append(leaf, "distance"), () -> val.get().distanceMeters);
        logRotation2d(level, append(leaf, "angle"), () -> val.get().angle);
    }

    @Override
    public void logArmAngles(Level level, String leaf, Supplier<ArmAngles> angles) {
        logDouble(level, append(leaf, "th1"), () -> angles.get().th1);
        logDouble(level, append(leaf, "th2"), () -> angles.get().th2);
    }

    @Override
    public void logState(Level level, String leaf, Supplier<State> state) {
        logPose2d(level, append(leaf, "pose"), () -> state.get().poseMeters);
        logDouble(level, append(leaf, "curvature"), () -> state.get().curvatureRadPerMeter);
        logDouble(level, append(leaf, "velocity"), () -> state.get().velocityMetersPerSecond);
        logDouble(level, append(leaf, "accel"), () -> state.get().accelerationMetersPerSecondSq);
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

    private void print(String key, String[] val) {
        if (!Telemetry.kAlsoPrint)
            return;
        Util.printf("%s: %s\n", key, Arrays.toString(val));
    }
}
package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        /** useful for normal operations */
        INFO(1),
        /** primarily useful during development */
        DEBUG(2);

        private int priority;

        private Level(int priority) {
            this.priority = priority;
        }

        boolean admit(Level other) {
            return this.priority >= other.priority;
        }
    }

    /**
     * useful for troubleshooting unit tests. it's quite slow.
     */
    private static final boolean kAlsoPrint = false;
    private static final Telemetry instance = new Telemetry();
    private final NetworkTableInstance inst;
    private final Map<String, Publisher> pubs;
    private final SendableChooser<Level> m_levelChooser;
    // avoids hitting sendable chooser mutex so often.
    private final Notifier m_levelUpdater;
    private Level m_level;

    /**
     * Uses the default network table instance.
     * Clients should use the static instance, not the constructor.
     */
    private Telemetry() {
        inst = NetworkTableInstance.getDefault();
        pubs = new HashMap<>();
        m_levelChooser = TelemetryLevelChooser.get();
        for (Level level : Level.values()) {
            m_levelChooser.addOption(level.name(), level);
        }
        m_levelChooser.setDefaultOption(Level.INFO.name(), Level.INFO);
        SmartDashboard.putData(m_levelChooser);
        updateLevel();
        m_levelUpdater = new Notifier(this::updateLevel);
        m_levelUpdater.startPeriodic(1);
        DataLogManager.start();
    }

    private void updateLevel() {
        m_level = m_levelChooser.getSelected();
    } 

    public static Telemetry get() {
        return instance;
    }

    public void log(Level level, String key, boolean val) {
        if (!m_level.admit(level))
            return;
        if (kAlsoPrint)
            Util.println(key + ": " + val);
        pub(key, k -> {
            BooleanTopic t = inst.getBooleanTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, BooleanPublisher.class).set(val);
    }

    public void log(Level level, String key, double val) {
        if (!m_level.admit(level))
            return;
        if (kAlsoPrint)
            Util.println(key + ": " + val);
        pub(key, k -> {
            DoubleTopic t = inst.getDoubleTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, DoublePublisher.class).set(val);
    }

    public void log(Level level, String key, double[] val) {
        if (!m_level.admit(level))
            return;
        if (kAlsoPrint)
            Util.println(key + ": " + val);
        pub(key, k -> {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, DoubleArrayPublisher.class).set(val);
    }

    public void log(Level level, String key, long val) {
        if (!m_level.admit(level))
            return;
        if (kAlsoPrint)
            Util.println(key + ": " + val);
        pub(key, k -> {
            IntegerTopic t = inst.getIntegerTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, IntegerPublisher.class).set(val);
    }

    public void log(Level level, String key, String val) {
        if (!m_level.admit(level))
            return;
        if (kAlsoPrint)
            Util.println(key + ": " + val);
        pub(key, k -> {
            StringTopic t = inst.getStringTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, StringPublisher.class).set(val);
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    public void log(Level level, String key, Supplier<String[]> val) {
        if (!m_level.admit(level))
            return;
        if (kAlsoPrint)
            Util.println(key + ": " + val);
        pub(key, k -> {
            StringArrayTopic t = inst.getStringArrayTopic(k);
            t.publish();
            t.setRetained(true);
            return t.publish();
        }, StringArrayPublisher.class).set(val.get());
    }

    public void log(Level level, String key, Pose2d val) {
        log(level, key + "/translation", val.getTranslation());
        log(level, key + "/rotation", val.getRotation());
    }

    public void log(Level level, String key, Translation2d val) {
        log(level, key + "/x", val.getX());
        log(level, key + "/y", val.getY());
    }

    public void log(Level level, String key, Rotation2d val) {
        log(level, key + "/rad", val.getRadians());
    }

    public void log(Level level, String key, TrajectorySamplePoint val) {
        log(level, key + "/state", val.state());
    }

    public void log(Level level, String key, TimedPose val) {
        log(level, key + "/posestate", val.state());
        log(level, key + "/time", val.getTimeS());
        log(level, key + "/velocity", val.velocityM_S());
        log(level, key + "/accel", val.acceleration());
    }

    public void log(Level level, String key, PoseWithCurvature val) {
        log(level, key + "/pose", val.poseMeters);
    }

    public void log(Level level, String key, Pose2dWithMotion val) {
        log(level, key + "/pose", val.getPose());
        Optional<Rotation2d> course = val.getCourse();
        if (course.isPresent()) {
            log(level, key + "/course", course.get());
        }
    }

    public void log(Level level, String key, Twist2d val) {
        log(level, key + "/dx", val.dx);
        log(level, key + "/dy", val.dy);
        log(level, key + "/dtheta", val.dtheta);
    }

    public void log(Level level, String key, ChassisSpeeds val) {
        log(level, key + "/vx m_s", val.vxMetersPerSecond);
        log(level, key + "/vy m_s", val.vyMetersPerSecond);
        log(level, key + "/omega rad_s", val.omegaRadiansPerSecond);
    }

    public void log(Level level, String key, State100 state) {
        log(level, key + "/x", state.x());
        log(level, key + "/v", state.v());
        log(level, key + "/a", state.a());
    }

    public void log(Level level, String key, SwerveState state) {
        log(level, key + "/x", state.x());
        log(level, key + "/y", state.y());
        log(level, key + "/theta", state.theta());
    }

    public void log(Level level, String key, ArmAngles angles) {
        log(level, key + "/th1", angles.th1);
        log(level, key + "/th2", angles.th2);
    }

    public void log(Level level, String key, State state) {
        log(level, key + "/pose", state.poseMeters);
        log(level, key + "/curvature", state.curvatureRadPerMeter);
        log(level, key + "/velocity", state.velocityMetersPerSecond);
        log(level, key + "/accel", state.accelerationMetersPerSecondSq);
    }

    private <T extends Publisher> T pub(String key, Function<String, Publisher> fn, Class<T> pubClass) {
        Publisher publisher = pubs.computeIfAbsent(valid(key), fn);
        if (!pubClass.isInstance(publisher))
            throw new IllegalArgumentException("value type clash");
        return pubClass.cast(publisher);
    }

    private String valid(String key) {
        if (key.length() == 0)
            throw new IllegalArgumentException("empty key");
        if (key.charAt(0) != '/')
            throw new IllegalArgumentException("no leading slash");
        return key;
    }
}

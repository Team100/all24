package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.PoseWithCurvature;
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
        INFO,
        /** primarily useful during development */
        DEBUG
    }

    /**
     * useful for troubleshooting unit tests. it's quite slow.
     */
    private static final boolean kAlsoPrint = false;
    private static final Telemetry instance = new Telemetry();
    private final NetworkTableInstance inst;
    private final Map<String, Publisher> pubs;
    private final SendableChooser<Level> m_manualModeChooser;

    /** Uses the default network table instance. */
    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        pubs = new HashMap<>();
        m_manualModeChooser = new SendableChooser<>();
        for (Level level : Level.values()) {
            m_manualModeChooser.addOption(level.name(), level);
        }
        m_manualModeChooser.setDefaultOption(
                Level.DEBUG.name(),
                Level.DEBUG);
        SmartDashboard.putData(m_manualModeChooser);

        DataLogManager.start();
    }

    public static Telemetry get() {
        return instance;
    }

    public void log(String key, boolean val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> {
            BooleanTopic t = inst.getBooleanTopic(k);
            t.setRetained(true);
            return t.publish();
        }, BooleanPublisher.class).set(val);
    }

    public void log(String key, double val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> {
            DoubleTopic t = inst.getDoubleTopic(k);
            t.setRetained(true);
            return t.publish();
        }, DoublePublisher.class).set(val);
    }

    public void log(String key, double[] val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(k);
            t.setRetained(true);
            return t.publish();
        }, DoubleArrayPublisher.class).set(val);
    }

    public void log(String key, long val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> {
            IntegerTopic t = inst.getIntegerTopic(k);
            t.setRetained(true);
            return t.publish();
        }, IntegerPublisher.class).set(val);
    }

    public void log(String key, String val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> {
            StringTopic t = inst.getStringTopic(k);
            t.setRetained(true);
            return t.publish();
        }, StringPublisher.class).set(val);
    }

    public void log(String key, String[] val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> {
            StringArrayTopic t = inst.getStringArrayTopic(k);
            t.setRetained(true);
            return t.publish();
        }, StringArrayPublisher.class).set(val);
    }

    public void log(String key, Pose2d val) {
        log(key + "/translation", val.getTranslation());
        log(key + "/rotation", val.getRotation());
    }

    public void log(String key, Translation2d val) {
        log(key + "/x", val.getX());
        log(key + "/y", val.getY());
    }

    public void log(String key, Rotation2d val) {
        log(key + "/rad", val.getRadians());
    }

    public void log(String key, TrajectorySamplePoint val) {
        log(key + "/state", val.state());
    }

    public void log(String key, TimedPose val) {
        log(key + "/posestate", val.state());
        log(key + "/time", val.getTimeS());
        log(key + "/velocity", val.velocityM_S());
        log(key + "/accel", val.acceleration());
    }

    public void log(String key, PoseWithCurvature val) {
        log(key + "/pose", val.poseMeters);
    }

    public void log(String key, Pose2dWithMotion val) {
        log(key + "/pose", val.getPose());
        Optional<Rotation2d> course = val.getCourse();
        if (course.isPresent()) {
            log(key + "/course", course.get());
        }

    }

    public void log(String key, Twist2d val) {
        log(key + "/dx", val.dx);
        log(key + "/dy", val.dy);
        log(key + "/dtheta", val.dtheta);
    }

    public void log(String key, ChassisSpeeds val) {
        log(key + "/vx m_s", val.vxMetersPerSecond);
        log(key + "/vy m_s", val.vyMetersPerSecond);
        log(key + "/omega rad_s", val.omegaRadiansPerSecond);

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

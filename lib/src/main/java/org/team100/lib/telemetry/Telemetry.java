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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Simple logging wrapper.
 * 
 * Use keys of the form "/foo/bar"; the slashes separate levels in the tree.
 * 
 * Don't use a slash for any other reason, e.g. for "meters per second" don't
 * say "m/s" say "m_s"
 */
public class Telemetry {
    /**
     * useful for troubleshooting unit tests.
     */
    private static final boolean kAlsoPrint = false;
    private static final Telemetry instance = new Telemetry();
    private final NetworkTableInstance inst;
    private final Map<String, Publisher> pubs;

    /** Uses the default network table instance. */
    public Telemetry() {
        inst = NetworkTableInstance.getDefault();
        pubs = new HashMap<>();
        DataLogManager.start();
    }

    public static Telemetry get() {
        return instance;
    }

    public void log(String key, boolean val) {
        if (kAlsoPrint)
            System.out.println(key + val);
        pub(key, k -> inst.getBooleanTopic(k).publish(), BooleanPublisher.class).set(val);
    }

    public void log(String key, double val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> inst.getDoubleTopic(k).publish(), DoublePublisher.class).set(val);
    }

    public void log(String key, double[] val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> inst.getDoubleArrayTopic(k).publish(), DoubleArrayPublisher.class).set(val);
    }

    public void log(String key, long val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> inst.getIntegerTopic(k).publish(), IntegerPublisher.class).set(val);
    }

    public void log(String key, String val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> inst.getStringTopic(k).publish(), StringPublisher.class).set(val);
    }

    public void log(String key, String[] val) {
        if (kAlsoPrint)
            System.out.println(key + ": " + val);
        pub(key, k -> inst.getStringArrayTopic(k).publish(), StringArrayPublisher.class).set(val);
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

    /** For testing. */
    Telemetry(NetworkTableInstance inst, Map<String, Publisher> pubs) {
        this.inst = inst;
        this.pubs = pubs;
    }
}

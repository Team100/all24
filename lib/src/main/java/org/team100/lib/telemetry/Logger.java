package org.team100.lib.telemetry;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.function.FloatSupplier;

/**
 * The logger interface uses suppliers to make the fast case (logging less)
 * fast.
 * 
 * All the methods have different names because Java erases the type parameter
 * of the suppliers.
 */
public interface Logger {

    /**
     * Use this to create a child logger that describes the purpose of the
     * subordinate thing, e.g. the "left" thing or the "right" thing.
     * 
     * Each child level is separated by slashes, to make a tree in glass.
     */
    Logger child(String stem);

    /**
     * Use this to create a child logger that describes the type of the subordinate
     * thing, using the stable glass name (i.e. interface names, not implementation
     * names, where possible).
     */
    default Logger child(Glassy obj) {
        return child(obj.getGlassName());
    }

    /** Make a key for the root level (with a leading slash). */
    default String append(String root, String leaf) {
        if (root.startsWith("/"))
            return root + "/" + leaf;
        return "/" + root + "/" + leaf;
    }

     void logBoolean(Level level, String leaf, BooleanSupplier val) ;

     void logDouble(Level level, String leaf, DoubleSupplier vals) ;

     void logInt(Level level, String leaf, IntSupplier vals);

     void logFloat(Level level, String leaf, FloatSupplier val) ;

     void logDoubleArray(Level level, String leaf, Supplier<double[]> val) ;

     void logDoubleObjArray(Level level, String leaf, Supplier<Double[]> val) ;

     void logLong(Level level, String leaf, LongSupplier val);

     void logString(Level level, String leaf, Supplier<String> val) ;

     void logStringArray(Level level, String leaf, Supplier<String[]> val) ;

    default void logOptionalDouble(Level level, String leaf, Supplier<OptionalDouble> vals) {
        OptionalDouble val = vals.get();
        if (val.isPresent()) {
            logDouble(level, leaf, val::getAsDouble);
        }
    }

    default void logEnum(Level level, String leaf, Supplier<Enum<?>> val) {
        logString(level, leaf, () -> val.get().name());
    }

    default void logPose2d(Level level, String leaf, Supplier<Pose2d> val) {
        logTranslation2d(level, append(leaf, "translation"), () -> val.get().getTranslation());
        logRotation2d(level, append(leaf, "rotation"), () -> val.get().getRotation());
    }

    default void logTranslation2d(Level level, String leaf, Supplier<Translation2d> val) {
        logDouble(level, append(leaf, "x"), () -> val.get().getX());
        logDouble(level, append(leaf, "y"), () -> val.get().getY());
    }

    default void logVector2d(Level level, String leaf, Supplier<Vector2d> val) {
        logDouble(level, append(leaf, "x"), () -> val.get().getX());
        logDouble(level, append(leaf, "y"), () -> val.get().getY());

    }

    default void logRotation2d(Level level, String leaf, Supplier<Rotation2d> val) {
        logDouble(level, append(leaf, "rad"), () -> val.get().getRadians());
    }

    default void logTrajectorySamplePoint(Level level, String leaf, Supplier<TrajectorySamplePoint> val) {
        logTimedPose(level, append(leaf, "state"), () -> val.get().state());
    }

    default void logTimedPose(Level level, String leaf, Supplier<TimedPose> val) {
        logPose2dWithMotion(level, append(leaf, "posestate"), () -> val.get().state());
        logDouble(level, append(leaf, "time"), () -> val.get().getTimeS());
        logDouble(level, append(leaf, "velocity"), () -> val.get().velocityM_S());
        logDouble(level, append(leaf, "accel"), () -> val.get().acceleration());
    }

    default void logPoseWithCurvature(Level level, String leaf, Supplier<PoseWithCurvature> val) {
        logPose2d(level, append(leaf, "pose"), () -> val.get().poseMeters);
    }

    default void logPose2dWithMotion(Level level, String leaf, Supplier<Pose2dWithMotion> val) {
        logPose2d(level, append(leaf, "pose"), () -> val.get().getPose());
        Optional<Rotation2d> course = val.get().getCourse();
        if (course.isPresent()) {
            logRotation2d(level, append(leaf, "course"), course::get);
        }
    }

    default void logTwist2d(Level level, String leaf, Supplier<Twist2d> val) {
        logDouble(level, append(leaf, "dx"), () -> val.get().dx);
        logDouble(level, append(leaf, "dy"), () -> val.get().dy);
        logDouble(level, append(leaf, "dtheta"), () -> val.get().dtheta);
    }

    default void logChassisSpeeds(Level level, String leaf, Supplier<ChassisSpeeds> val) {
        logDouble(level, append(leaf, "vx m_s"), () -> val.get().vxMetersPerSecond);
        logDouble(level, append(leaf, "vy m_s"), () -> val.get().vyMetersPerSecond);
        logDouble(level, append(leaf, "omega rad_s"), () -> val.get().omegaRadiansPerSecond);
    }

    default void logFieldRelativeVelocity(Level level, String leaf, Supplier<FieldRelativeVelocity> val) {
        logDouble(level, append(leaf, "x m_s"), () -> val.get().x());
        logDouble(level, append(leaf, "y m_s"), () -> val.get().y());
        logDouble(level, append(leaf, "theta rad_s"), () -> val.get().theta());
    }

    default void logFieldRelativeAcceleration(Level level, String leaf, Supplier<FieldRelativeAcceleration> val) {
        logDouble(level, append(leaf, "x m_s_s"), () -> val.get().x());
        logDouble(level, append(leaf, "y m_s_s"), () -> val.get().y());
        logDouble(level, append(leaf, "theta rad_s_s"), () -> val.get().theta());
    }

    default void logState100(Level level, String leaf, Supplier<State100> state) {
        logDouble(level, append(leaf, "x"), () -> state.get().x());
        logDouble(level, append(leaf, "v"), () -> state.get().v());
        logDouble(level, append(leaf, "a"), () -> state.get().a());
    }

    default void logSwerveState(Level level, String leaf, Supplier<SwerveState> state) {
        logState100(level, append(leaf, "x"), () -> state.get().x());
        logState100(level, append(leaf, "y"), () -> state.get().y());
        logState100(level, append(leaf, "theta"), () -> state.get().theta());
    }

    default void logSwerveModulePosition(Level level, String leaf, Supplier<SwerveModulePosition> val) {
        logDouble(level, append(leaf, "distance"), () -> val.get().distanceMeters);
        logRotation2d(level, append(leaf, "angle"), () -> val.get().angle);
    }

    default void logArmAngles(Level level, String leaf, Supplier<ArmAngles> angles) {
        logDouble(level, append(leaf, "th1"), () -> angles.get().th1);
        logDouble(level, append(leaf, "th2"), () -> angles.get().th2);
    }

    default void logState(Level level, String leaf, Supplier<State> state) {
        logPose2d(level, append(leaf, "pose"), () -> state.get().poseMeters);
        logDouble(level, append(leaf, "curvature"), () -> state.get().curvatureRadPerMeter);
        logDouble(level, append(leaf, "velocity"), () -> state.get().velocityMetersPerSecond);
        logDouble(level, append(leaf, "accel"), () -> state.get().accelerationMetersPerSecondSq);
    }
}
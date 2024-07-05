package org.team100.lib.telemetry;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
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

    /** Adds a slash between the root and the stem */
    Logger child(String stem);

    default Logger child(Glassy obj) {
        return child(obj.getGlassName());
    }

    default Logger child(Class<?> clazz) {
        return child(clazz.getSimpleName());
    }

    default boolean enabled() {
        return false;
    }

    default void logBoolean(Level level, String leaf, BooleanSupplier val) {
    }

    /**
     * This is for tuning through glass.
     * Remember that the values don't survive restarts, so
     * you should write them down.
     */
    default void register(Level level, String leaf, double initial, DoubleConsumer consumer) {
    }

    default void logDouble(Level level, String leaf, DoubleSupplier vals) {
    }

    default void logInt(Level level, String leaf, IntSupplier vals) {
    }

    default void logOptionalDouble(Level level, String leaf, Supplier<OptionalDouble> val) {
    }

    default void logFloat(Level level, String leaf, FloatSupplier val) {
    }

    default void logDoubleArray(Level level, String leaf, Supplier<double[]> val) {
    }

    default void logDoubleObjArray(Level level, String leaf, Supplier<Double[]> val) {
    }

    default void loglong(Level level, String leaf, LongSupplier val) {
    }

    default void logString(Level level, String leaf, Supplier<String> val) {
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    default void logStringArray(Level level, String leaf, Supplier<String[]> val) {
    }

    default void logEnum(Level level, String leaf, Supplier<Enum<?>> val) {
    }

    default void logPose2d(Level level, String leaf, Supplier<Pose2d> val) {
    }

    default void logTranslation2d(Level level, String leaf, Supplier<Translation2d> val) {
    }

    default void logVector2d(Level level, String leaf, Supplier<Vector2d> val) {
    }

    default void logRotation2d(Level level, String leaf, Supplier<Rotation2d> val) {
    }

    default void logTrajectorySamplePoint(Level level, String leaf, Supplier<TrajectorySamplePoint> val) {
    }

    default void logTimedPose(Level level, String leaf, Supplier<TimedPose> val) {
    }

    default void logPoseWithCurvature(Level level, String leaf, Supplier<PoseWithCurvature> val) {
    }

    default void logPose2dWithMotion(Level level, String leaf, Supplier<Pose2dWithMotion> val) {
    }

    default void logTwist2d(Level level, String leaf, Supplier<Twist2d> val) {
    }

    default void logChassisSpeeds(Level level, String leaf, Supplier<ChassisSpeeds> val) {
    }

    default void logFieldRelativeVelocity(Level level, String leaf, Supplier<FieldRelativeVelocity> val) {
    }

    default void logFieldRelativeAcceleration(Level level, String leaf, Supplier<FieldRelativeAcceleration> val) {
    }

    default void logState100(Level level, String leaf, Supplier<State100> state) {
    }

    default void logSwerveState(Level level, String leaf, Supplier<SwerveState> state) {
    }

    default void logSwerveModulePosition(Level level, String leaf, Supplier<SwerveModulePosition> val) {
    }

    default void logArmAngles(Level level, String leaf, Supplier<ArmAngles> angles) {
    }

    default void logState(Level level, String leaf, Supplier<State> state) {
    }

}
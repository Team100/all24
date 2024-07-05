package org.team100.lib.telemetry;

import java.util.OptionalDouble;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
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

    default void logBoolean(Level level, String leaf, boolean val) {
    }

    /**
     * This is for tuning through glass.
     * Remember that the values don't survive restarts, so
     * you should write them down.
     */
    default void register(Level level, String leaf, double initial, DoubleConsumer consumer) {
    }

    // using a supplier here is faster in the non-logging case.
    default void logDouble(Level level, String leaf, DoubleSupplier vals) {
    }

    default void log(Level level, String leaf, OptionalDouble val) {
    }

    default void log(Level level, String leaf, float val) {
    }

    default void log(Level level, String leaf, double[] val) {
    }

    default void log(Level level, String leaf, Double[] val) {
    }

    default void log(Level level, String leaf, long val) {
    }

    default void log(Level level, String leaf, String val) {
    }

    /** val is a supplier to avoid doing any work if we're not going to log it. */
    default void log(Level level, String leaf, Supplier<String[]> val) {
    }

    default void log(Level level, String leaf, Enum<?> val) {
    }

    default void log(Level level, String leaf, Pose2d val) {
    }

    default void log(Level level, String leaf, Translation2d val) {
    }

    default void log(Level level, String leaf, Vector2d val) {
    }

    default void log(Level level, String leaf, Rotation2d val) {
    }

    default void log(Level level, String leaf, TrajectorySamplePoint val) {
    }

    default void log(Level level, String leaf, TimedPose val) {
    }

    default void log(Level level, String leaf, PoseWithCurvature val) {
    }

    default void log(Level level, String leaf, Pose2dWithMotion val) {
    }

    default void log(Level level, String leaf, Twist2d val) {
    }

    default void log(Level level, String leaf, ChassisSpeeds val) {
    }

    default void log(Level level, String leaf, FieldRelativeVelocity val) {
    }

    default void log(Level level, String leaf, FieldRelativeAcceleration val) {
    }

    default void log(Level level, String leaf, State100 state) {
    }

    default void log(Level level, String leaf, SwerveState state) {
    }

    default void log(Level level, String leaf, SwerveModulePosition val) {
    }

    default void log(Level level, String leaf, ArmAngles angles) {
    }

    default void log(Level level, String leaf, State state) {
    }

}
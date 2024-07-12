package org.team100.lib.telemetry;

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
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectorySamplePoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.function.FloatSupplier;

/**
 * The logger interface uses suppliers to make the fast case (logging less)
 * fast.
 * 
 * All the methods have different names because Java erases the type parameter
 * of the suppliers.
 * 
 * It's an interface with defaults so that the test mock is simple.
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

    default void logBoolean(Level level, String leaf, BooleanSupplier val) {
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

    default void logLong(Level level, String leaf, LongSupplier val) {
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
    
    default void logSwerveModulePosition100(Level level, String leaf, Supplier<SwerveModulePosition100> val) {
    }

    default void logArmAngles(Level level, String leaf, Supplier<ArmAngles> angles) {
    }

    default void logState(Level level, String leaf, Supplier<State> state) {
    }

}
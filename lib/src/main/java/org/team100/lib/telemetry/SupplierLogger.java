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
import org.team100.lib.localization.Blip24;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.function.FloatSupplier;

/**
 * Avoids touching the suppliers if not enabled.
 * 
 * Each primitive logger is independently selectable.
 */
public class SupplierLogger {
    private final Telemetry m_telemetry;
    private final String m_root;
    private final BooleanSupplier m_enabledA;
    private final PrimitiveLogger m_primitiveLoggerA;
    private final BooleanSupplier m_enabledB;
    private final PrimitiveLogger m_primitiveLoggerB;

    public SupplierLogger(
            Telemetry telemetry,
            String root,
            BooleanSupplier enabledA,
            PrimitiveLogger primitiveLoggerA,
            BooleanSupplier enabledB,
            PrimitiveLogger primitiveLoggerB) {
        m_telemetry = telemetry;
        m_root = root;
        m_enabledA = enabledA;
        m_primitiveLoggerA = primitiveLoggerA;
        m_enabledB = enabledB;
        m_primitiveLoggerB = primitiveLoggerB;
    }

    /**
     * Use this to create a child logger that describes the purpose of the
     * subordinate thing, e.g. the "left" thing or the "right" thing.
     * 
     * Each child level is separated by slashes, to make a tree in glass.
     */
    public SupplierLogger child(String stem) {
        return new SupplierLogger(m_telemetry, m_root + "/" + stem,
                m_enabledA, m_primitiveLoggerA, m_enabledB, m_primitiveLoggerB);
    }

    /**
     * Use this to create a child logger that describes the type of the subordinate
     * thing, using the stable glass name (i.e. interface names, not implementation
     * names, where possible).
     */
    public SupplierLogger child(Glassy obj) {
        return child(obj.getGlassName());
    }

    private boolean allow(Level level) {
        // if (m_telemetry.getLoadShedder().expired()) {
        //     // if we're out of time for logging, don't do it.
        //     return false;
        // }
        if (m_telemetry.getLevel() == Level.COMP && level == Level.COMP) {
            // comp mode allows COMP level regardless of enablement.
            return true;
        }
        if (!m_telemetry.getLevel().admit(level)) {
            // if level is too low, enablement doesn't matter.
            return false;
        }
        // either one means unpacking the supplier
        return enabledA() || enabledB();
    }

    private boolean enabledA() {
        return m_enabledA.getAsBoolean();
    }

    private boolean enabledB() {
        return m_enabledB.getAsBoolean();
    }

    /** Make a key for the root level (with a leading slash). */
    private String append(String root, String leaf) {
        if (root.startsWith("/"))
            return root + "/" + leaf;
        return "/" + root + "/" + leaf;
    }

    public void logBoolean(Level level, String leaf, BooleanSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        boolean val = vals.getAsBoolean();
        if (enabledA())
            m_primitiveLoggerA.logBoolean(key, val);
        if (enabledB())
            m_primitiveLoggerB.logBoolean(key, val);
    }

    public void logDouble(Level level, String leaf, DoubleSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        double val = vals.getAsDouble();
        if (enabledA())
            m_primitiveLoggerA.logDouble(key, val);
        if (enabledB())
            m_primitiveLoggerB.logDouble(key, val);
    }

    public void logInt(Level level, String leaf, IntSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        int val = vals.getAsInt();
        if (enabledA())
            m_primitiveLoggerA.logInt(key, val);
        if (enabledB())
            m_primitiveLoggerB.logInt(key, val);
    }

    public void logDoubleArray(Level level, String leaf, Supplier<double[]> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        double[] val = vals.get();
        if (enabledA())
            m_primitiveLoggerA.logDoubleArray(key, val);
        if (enabledB())
            m_primitiveLoggerB.logDoubleArray(key, val);
    }

    public void logDoubleObjArray(Level level, String leaf, Supplier<Double[]> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        Double[] val = vals.get();
        if (enabledA())
            m_primitiveLoggerA.logDoubleObjArray(key, val);
        if (enabledB())
            m_primitiveLoggerB.logDoubleObjArray(key, val);
    }

    public void logLong(Level level, String leaf, LongSupplier vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        long val = vals.getAsLong();
        if (enabledA())
            m_primitiveLoggerA.logLong(key, val);
        if (enabledB())
            m_primitiveLoggerB.logLong(key, val);
    }

    public void logString(Level level, String leaf, Supplier<String> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        String val = vals.get();
        if (enabledA())
            m_primitiveLoggerA.logString(key, val);
        if (enabledB())
            m_primitiveLoggerB.logString(key, val);
    }

    public void logOptionalDouble(Level level, String leaf, Supplier<OptionalDouble> vals) {
        if (!allow(level))
            return;
        OptionalDouble val = vals.get();
        if (val.isPresent()) {
            String key = append(m_root, leaf);
            if (enabledA())
                m_primitiveLoggerA.logDouble(key, val.getAsDouble());
            if (enabledB())
                m_primitiveLoggerB.logDouble(key, val.getAsDouble());
        }
    }

    public void logEnum(Level level, String leaf, Supplier<Enum<?>> vals) {
        if (!allow(level))
            return;
        String key = append(m_root, leaf);
        String val = vals.get().name();
        if (enabledA())
            m_primitiveLoggerA.logString(key, val);
        if (enabledB())
            m_primitiveLoggerB.logString(key, val);
    }

    public void logPose2d(Level level, String leaf, Supplier<Pose2d> vals) {
        if (!allow(level))
            return;
        Pose2d pose2d = vals.get();
        logTranslation2d(level, append(leaf, "translation"), pose2d::getTranslation);
        logRotation2d(level, append(leaf, "rotation"), pose2d::getRotation);
    }

    public void logTransform3d(Level level, String leaf, Supplier<Transform3d> vals) {
        if (!allow(level))
            return;
        Transform3d transform3d = vals.get();
        logTranslation3d(level, append(leaf, "translation"), transform3d::getTranslation);
        logRotation3d(level, append(leaf, "rotation"), transform3d::getRotation);
    }

    public void logTranslation3d(Level level, String leaf, Supplier<Translation3d> vals) {
        if (!allow(level))
            return;
        Translation3d translation3d = vals.get();
        logDouble(level, append(leaf, "x"), translation3d::getX);
        logDouble(level, append(leaf, "y"), translation3d::getY);
        logDouble(level, append(leaf, "z"), translation3d::getZ);
    }

    public void logRotation3d(Level level, String leaf, Supplier<Rotation3d> vals) {
        if (!allow(level))
            return;
        Rotation3d rotation3d = vals.get();
        logDouble(level, append(leaf, "roll"), rotation3d::getX);
        logDouble(level, append(leaf, "pitch"), rotation3d::getY);
        logDouble(level, append(leaf, "yaw"), rotation3d::getZ);
    }

    public void logTranslation2d(Level level, String leaf, Supplier<Translation2d> vals) {
        if (!allow(level))
            return;
        Translation2d translation2d = vals.get();
        logDouble(level, append(leaf, "x"), translation2d::getX);
        logDouble(level, append(leaf, "y"), translation2d::getY);
    }

    public void logVector2d(Level level, String leaf, Supplier<Vector2d> vals) {
        if (!allow(level))
            return;
        Vector2d vector2d = vals.get();
        logDouble(level, append(leaf, "x"), vector2d::getX);
        logDouble(level, append(leaf, "y"), vector2d::getY);
    }

    public void logRotation2d(Level level, String leaf, Supplier<Rotation2d> vals) {
        if (!allow(level))
            return;
        Rotation2d rotation2d = vals.get();
        logDouble(level, append(leaf, "rad"), rotation2d::getRadians);
    }

    public void logTrajectorySamplePoint(Level level, String leaf, Supplier<TrajectorySamplePoint> vals) {
        if (!allow(level))
            return;
        TrajectorySamplePoint trajectorySamplePoint = vals.get();
        logTimedPose(level, append(leaf, "state"), trajectorySamplePoint::state);
    }

    public void logTimedPose(Level level, String leaf, Supplier<TimedPose> vals) {
        if (!allow(level))
            return;
        TimedPose timedPose = vals.get();
        logPose2dWithMotion(level, append(leaf, "posestate"), timedPose::state);
        logDouble(level, append(leaf, "time"), timedPose::getTimeS);
        logDouble(level, append(leaf, "velocity"), timedPose::velocityM_S);
        logDouble(level, append(leaf, "accel"), timedPose::acceleration);
    }

    public void logPoseWithCurvature(Level level, String leaf, Supplier<PoseWithCurvature> vals) {
        if (!allow(level))
            return;
        PoseWithCurvature poseWithCurvature = vals.get();
        logPose2d(level, append(leaf, "pose"), () -> poseWithCurvature.poseMeters);
    }

    public void logPose2dWithMotion(Level level, String leaf, Supplier<Pose2dWithMotion> vals) {
        if (!allow(level))
            return;
        Pose2dWithMotion pose2dWithMotion = vals.get();
        logPose2d(level, append(leaf, "pose"), pose2dWithMotion::getPose);
        Optional<Rotation2d> course = pose2dWithMotion.getCourse();
        if (course.isPresent()) {
            logRotation2d(level, append(leaf, "course"), course::get);
        }
    }

    public void logTwist2d(Level level, String leaf, Supplier<Twist2d> vals) {
        if (!allow(level))
            return;
        Twist2d twist2d = vals.get();
        logDouble(level, append(leaf, "dx"), () -> twist2d.dx);
        logDouble(level, append(leaf, "dy"), () -> twist2d.dy);
        logDouble(level, append(leaf, "dtheta"), () -> twist2d.dtheta);
    }

    public void logChassisSpeeds(Level level, String leaf, Supplier<ChassisSpeeds> vals) {
        if (!allow(level))
            return;
        ChassisSpeeds chassisSpeeds = vals.get();
        logDouble(level, append(leaf, "vx m_s"), () -> chassisSpeeds.vxMetersPerSecond);
        logDouble(level, append(leaf, "vy m_s"), () -> chassisSpeeds.vyMetersPerSecond);
        logDouble(level, append(leaf, "omega rad_s"), () -> chassisSpeeds.omegaRadiansPerSecond);
    }

    public void logFieldRelativeVelocity(Level level, String leaf, Supplier<FieldRelativeVelocity> vals) {
        if (!allow(level))
            return;
        FieldRelativeVelocity fieldRelativeVelocity = vals.get();
        logDouble(level, append(leaf, "x m_s"), fieldRelativeVelocity::x);
        logDouble(level, append(leaf, "y m_s"), fieldRelativeVelocity::y);
        logDouble(level, append(leaf, "theta rad_s"), fieldRelativeVelocity::theta);
    }

    public void logFieldRelativeAcceleration(Level level, String leaf, Supplier<FieldRelativeAcceleration> vals) {
        if (!allow(level))
            return;
        FieldRelativeAcceleration fieldRelativeAcceleration = vals.get();
        logDouble(level, append(leaf, "x m_s_s"), fieldRelativeAcceleration::x);
        logDouble(level, append(leaf, "y m_s_s"), fieldRelativeAcceleration::y);
        logDouble(level, append(leaf, "theta rad_s_s"), fieldRelativeAcceleration::theta);
    }

    public void logState100(Level level, String leaf, Supplier<State100> vals) {
        if (!allow(level))
            return;
        State100 state100 = vals.get();
        logDouble(level, append(leaf, "x"), state100::x);
        logDouble(level, append(leaf, "v"), state100::v);
        logDouble(level, append(leaf, "a"), state100::a);
    }

    public void logSwerveState(Level level, String leaf, Supplier<SwerveState> vals) {
        if (!allow(level))
            return;
        SwerveState swerveState = vals.get();
        logState100(level, append(leaf, "x"), swerveState::x);
        logState100(level, append(leaf, "y"), swerveState::y);
        logState100(level, append(leaf, "theta"), swerveState::theta);
    }

    public void logSwerveModulePosition100(Level level, String leaf, Supplier<SwerveModulePosition100> vals) {
        if (!allow(level))
            return;
        SwerveModulePosition100 swerveModulePosition100 = vals.get();
        logDouble(level, append(leaf, "distance"), () -> swerveModulePosition100.distanceMeters);
        if (swerveModulePosition100.angle.isPresent()) {
            logRotation2d(level, append(leaf, "angle"), swerveModulePosition100.angle::get);
        }
    }

    public void logArmAngles(Level level, String leaf, Supplier<ArmAngles> vals) {
        if (!allow(level))
            return;
        ArmAngles armAngles = vals.get();
        logDouble(level, append(leaf, "th1"), () -> armAngles.th1);
        logDouble(level, append(leaf, "th2"), () -> armAngles.th2);
    }

    public void logState(Level level, String leaf, Supplier<State> vals) {
        if (!allow(level))
            return;
        State state = vals.get();
        logPose2d(level, append(leaf, "pose"), () -> state.poseMeters);
        logDouble(level, append(leaf, "curvature"), () -> state.curvatureRadPerMeter);
        logDouble(level, append(leaf, "velocity"), () -> state.velocityMetersPerSecond);
        logDouble(level, append(leaf, "accel"), () -> state.accelerationMetersPerSecondSq);
    }

    public void logBlip24(Level level, String leaf, Supplier<Blip24> vals) {
        if (!allow(level))
            return;
        Blip24 blip = vals.get();
        logInt(level, append(leaf, "id"), blip::getId);
        logTransform3d(level, append(leaf, "transform"), blip::getPose);
    }

}

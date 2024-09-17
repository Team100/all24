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

public class SupplierLogger2 {
    private final Telemetry m_telemetry;
    private final String m_root;
    private final PrimitiveLogger2 m_primitiveLoggerA;

    public SupplierLogger2(
            Telemetry telemetry,
            String root,
            PrimitiveLogger2 primitiveLoggerA) {
        m_telemetry = telemetry;
        m_root = root;
        m_primitiveLoggerA = primitiveLoggerA;
    }

    /**
     * Use this to create a child logger that describes the purpose of the
     * subordinate thing, e.g. the "left" thing or the "right" thing.
     * 
     * Each child level is separated by slashes, to make a tree in glass.
     */
    public SupplierLogger2 child(String stem) {
        return new SupplierLogger2(m_telemetry, m_root + "/" + stem,
                m_primitiveLoggerA);
    }

    /**
     * Use this to create a child logger that describes the type of the subordinate
     * thing, using the stable glass name (i.e. interface names, not implementation
     * names, where possible).
     */
    public SupplierLogger2 child(Glassy obj) {
        return child(obj.getGlassName());
    }

    private boolean allow(Level level) {
        if (m_telemetry.getLoadShedder().expired()) {
            // if we're out of time for logging, don't do it.
            return false;
        }
        if (m_telemetry.getLevel() == Level.COMP && level == Level.COMP) {
            // comp mode allows COMP level regardless of enablement.
            return true;
        }
        return m_telemetry.getLevel().admit(level);
    }

    /** Make a key for the root level (with a leading slash). */
    private String append(String root, String leaf) {
        if (root.startsWith("/"))
            return root + "/" + leaf;
        return "/" + root + "/" + leaf;
    }

    public class BooleanSupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.BooleanLogger m_primitiveLogger;

        BooleanSupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.booleanLogger(append(m_root, leaf));
        }

        public void log(BooleanSupplier vals) {
            if (!allow(m_level))
                return;
            boolean val = vals.getAsBoolean();
            m_primitiveLogger.log(val);
        }
    }

    public BooleanSupplierLogger booleanLogger(Level level, String leaf) {
        return new BooleanSupplierLogger(level, leaf);
    }

    public class DoubleSupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.DoubleLogger m_primitiveLogger;

        DoubleSupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.doubleLogger(append(m_root, leaf));
        }

        public void log(DoubleSupplier vals) {
            // if (!allow(m_level))
            //     return;
            double val = vals.getAsDouble();
            m_primitiveLogger.log(val);
        }
    }

    public DoubleSupplierLogger doubleLogger(Level level, String leaf) {
        return new DoubleSupplierLogger(level, leaf);
    }

    public class IntSupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.IntLogger m_primitiveLogger;

        IntSupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.intLogger(append(m_root, leaf));
        }

        public void log(IntSupplier vals) {
            if (!allow(m_level))
                return;
            int val = vals.getAsInt();
            m_primitiveLogger.log(val);
        }
    }

    public IntSupplierLogger intLogger(Level level, String leaf) {
        return new IntSupplierLogger(level, leaf);
    }

    public class DoubleArraySupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.DoubleArrayLogger m_primitiveLogger;

        DoubleArraySupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.doubleArrayLogger(append(m_root, leaf));
        }

        public void log(Supplier<double[]> vals) {
            if (!allow(m_level))
                return;
            double[] val = vals.get();
            m_primitiveLogger.log(val);
        }
    }

    public DoubleArraySupplierLogger doubleArrayLogger(Level level, String leaf) {
        return new DoubleArraySupplierLogger(level, leaf);
    }

    public class DoubleObjArraySupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.DoubleObjArrayLogger m_primitiveLogger;

        DoubleObjArraySupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.doubleObjArrayLogger(append(m_root, leaf));
        }

        public void log(Supplier<Double[]> vals) {
            if (!allow(m_level))
                return;
            Double[] val = vals.get();
            m_primitiveLogger.log(val);
        }
    }

    public DoubleObjArraySupplierLogger doubleObjArrayLogger(Level level, String leaf) {
        return new DoubleObjArraySupplierLogger(level, leaf);
    }

    public class LongSupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.LongLogger m_primitiveLogger;

        LongSupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.longLogger(append(m_root, leaf));
        }

        public void log(LongSupplier vals) {
            if (!allow(m_level))
                return;
            long val = vals.getAsLong();
            m_primitiveLogger.log(val);
        }
    }

    public LongSupplierLogger longLogger(Level level, String leaf) {
        return new LongSupplierLogger(level, leaf);
    }

    public class StringSupplierLogger {
        private final Level m_level;
        private final PrimitiveLogger2.StringLogger m_primitiveLogger;

        StringSupplierLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.stringLogger(append(m_root, leaf));
        }

        public void log(Supplier<String> vals) {
            if (!allow(m_level))
                return;
            String val = vals.get();
            m_primitiveLogger.log(val);
        }
    }

    public StringSupplierLogger stringLogger(Level level, String leaf) {
        return new StringSupplierLogger(level, leaf);
    }

    public class OptionalDoubleLogger {
        private final Level m_level;
        private final PrimitiveLogger2.DoubleLogger m_primitiveLogger;

        OptionalDoubleLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.doubleLogger(append(m_root, leaf));
        }

        public void log(Supplier<OptionalDouble> vals) {
            if (!allow(m_level))
                return;
            OptionalDouble val = vals.get();
            if (val.isPresent()) {
                m_primitiveLogger.log(val.getAsDouble());
            }
        }
    }

    public OptionalDoubleLogger optionalDoubleLogger(Level level, String leaf) {
        return new OptionalDoubleLogger(level, leaf);
    }

    public class EnumLogger {
        private final Level m_level;
        private final PrimitiveLogger2.StringLogger m_primitiveLogger;

        EnumLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_primitiveLoggerA.stringLogger(append(m_root, leaf));
        }

        public void log(Supplier<Enum<?>> vals) {
            if (!allow(m_level))
                return;
            String val = vals.get().name();
            m_primitiveLogger.log(val);
        }
    }

    public EnumLogger enumLogger(Level level, String leaf) {
        return new EnumLogger(level, leaf);
    }

    public class Pose2dLogger {
        private final Level m_level;
        private final Translation2dLogger m_translation2dLogger;
        private final Rotation2dLogger m_rotation2dLogger;

        Pose2dLogger(Level level, String leaf) {
            m_level = level;
            m_translation2dLogger = translation2dLogger(level, append(leaf, "translation"));
            m_rotation2dLogger = rotation2dLogger(level, append(leaf, "rotation"));
        }

        public void log(Supplier<Pose2d> vals) {
            if (!allow(m_level))
                return;
            Pose2d val = vals.get();
            m_translation2dLogger.log(val::getTranslation);
            m_rotation2dLogger.log(val::getRotation);
        }
    }

    public Pose2dLogger pose2dLogger(Level level, String leaf) {
        return new Pose2dLogger(level, leaf);
    }

    public class Transform3dLogger {
        private final Level m_level;
        private final Translation3dLogger m_translation3dLogger;
        private final Rotation3dLogger m_rotation3dLogger;

        Transform3dLogger(Level level, String leaf) {
            m_level = level;
            m_translation3dLogger = translation3dLogger(level, append(leaf, "translation"));
            m_rotation3dLogger = rotation3dLogger(level, append(leaf, "rotation"));
        }

        public void log(Supplier<Transform3d> vals) {
            if (!allow(m_level))
                return;
            Transform3d val = vals.get();
            m_translation3dLogger.log(val::getTranslation);
            m_rotation3dLogger.log(val::getRotation);
        }
    }

    public Transform3dLogger transform3dLogger(Level level, String leaf) {
        return new Transform3dLogger(level, leaf);
    }

    public class Translation3dLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_xLogger;
        private final DoubleSupplierLogger m_yLogger;
        private final DoubleSupplierLogger m_zLogger;

        Translation3dLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, append(leaf, "x"));
            m_yLogger = doubleLogger(level, append(leaf, "y"));
            m_zLogger = doubleLogger(level, append(leaf, "z"));
        }

        public void log(Supplier<Translation3d> vals) {
            if (!allow(m_level))
                return;
            Translation3d val = vals.get();
            m_xLogger.log(val::getX);
            m_yLogger.log(val::getY);
            m_zLogger.log(val::getZ);
        }
    }

    public Translation3dLogger translation3dLogger(Level level, String leaf) {
        return new Translation3dLogger(level, leaf);
    }

    public class Rotation3dLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_rollLogger;
        private final DoubleSupplierLogger m_pitchLogger;
        private final DoubleSupplierLogger m_yawLogger;

        Rotation3dLogger(Level level, String leaf) {
            m_level = level;
            m_rollLogger = doubleLogger(level, append(leaf, "roll"));
            m_pitchLogger = doubleLogger(level, append(leaf, "pitch"));
            m_yawLogger = doubleLogger(level, append(leaf, "yaw"));
        }

        public void log(Supplier<Rotation3d> vals) {
            if (!allow(m_level))
                return;
            Rotation3d val = vals.get();
            m_rollLogger.log(val::getX);
            m_pitchLogger.log(val::getY);
            m_yawLogger.log(val::getZ);
        }
    }

    public Rotation3dLogger rotation3dLogger(Level level, String leaf) {
        return new Rotation3dLogger(level, leaf);
    }

    public class Translation2dLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_xLogger;
        private final DoubleSupplierLogger m_yLogger;

        Translation2dLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, append(leaf, "x"));
            m_yLogger = doubleLogger(level, append(leaf, "y"));
        }

        public void log(Supplier<Translation2d> vals) {
            if (!allow(m_level))
                return;
            Translation2d val = vals.get();
            m_xLogger.log(val::getX);
            m_yLogger.log(val::getY);
        }
    }

    public Translation2dLogger translation2dLogger(Level level, String leaf) {
        return new Translation2dLogger(level, leaf);
    }

    public class Vector2dLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_xLogger;
        private final DoubleSupplierLogger m_yLogger;

        Vector2dLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, append(leaf, "x"));
            m_yLogger = doubleLogger(level, append(leaf, "y"));
        }

        public void log(Supplier<Vector2d> vals) {
            if (!allow(m_level))
                return;
            Vector2d val = vals.get();
            m_xLogger.log(val::getX);
            m_yLogger.log(val::getY);
        }
    }

    public Vector2dLogger vector2dLogger(Level level, String leaf) {
        return new Vector2dLogger(level, leaf);
    }

    public class Rotation2dLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_radLogger;

        Rotation2dLogger(Level level, String leaf) {
            m_level = level;
            m_radLogger = doubleLogger(level, append(leaf, "rad"));
        }

        public void log(Supplier<Rotation2d> vals) {
            if (!allow(m_level))
                return;
            Rotation2d val = vals.get();
            m_radLogger.log(val::getRadians);
        }
    }

    public Rotation2dLogger rotation2dLogger(Level level, String leaf) {
        return new Rotation2dLogger(level, leaf);
    }

    public class TrajectorySamplePointLogger {
        private final Level m_level;
        private final TimedPoseLogger m_timedPoseLogger;

        TrajectorySamplePointLogger(Level level, String leaf) {
            m_level = level;
            m_timedPoseLogger = timedPoseLogger(level, append(leaf, "state"));
        }

        public void log(Supplier<TrajectorySamplePoint> vals) {
            if (!allow(m_level))
                return;
            TrajectorySamplePoint val = vals.get();
            m_timedPoseLogger.log(val::state);
        }
    }

    public TrajectorySamplePointLogger trajectorySamplePointLogger(Level level, String leaf) {
        return new TrajectorySamplePointLogger(level, leaf);
    }

    public class TimedPoseLogger {
        private final Level m_level;
        private final Pose2dWithMotionLogger m_pose2dWithMotionLogger;
        private final DoubleSupplierLogger m_timeLogger;
        private final DoubleSupplierLogger m_velocityLogger;
        private final DoubleSupplierLogger m_accelLogger;

        TimedPoseLogger(Level level, String leaf) {
            m_level = level;
            m_pose2dWithMotionLogger = pose2dWithMotionLogger(level, append(leaf, "posestate"));
            m_timeLogger = doubleLogger(level, append(leaf, "time"));
            m_velocityLogger = doubleLogger(level, append(leaf, "velocity"));
            m_accelLogger = doubleLogger(level, append(leaf, "accel"));
        }

        public void log(Supplier<TimedPose> vals) {
            if (!allow(m_level))
                return;
            TimedPose val = vals.get();
            m_pose2dWithMotionLogger.log(val::state);
            m_timeLogger.log(val::getTimeS);
            m_velocityLogger.log(val::velocityM_S);
            m_accelLogger.log(val::acceleration);

        }
    }

    public TimedPoseLogger timedPoseLogger(Level level, String leaf) {
        return new TimedPoseLogger(level, leaf);
    }

    public class PoseWithCurvatureLogger {
        private final Level m_level;
        private final Pose2dLogger m_pose2dLogger;

        PoseWithCurvatureLogger(Level level, String leaf) {
            m_level = level;
            m_pose2dLogger = pose2dLogger(level, append(leaf, "pose"));
        }

        public void log(Supplier<PoseWithCurvature> vals) {
            if (!allow(m_level))
                return;
            PoseWithCurvature val = vals.get();
            m_pose2dLogger.log(() -> val.poseMeters);
        }
    }

    public PoseWithCurvatureLogger poseWithCurvatureLogger(Level level, String leaf) {
        return new PoseWithCurvatureLogger(level, leaf);
    }

    public class Pose2dWithMotionLogger {
        private final Level m_level;
        private final Pose2dLogger m_pose2dLogger;
        private final Rotation2dLogger m_rotation2dLogger;

        Pose2dWithMotionLogger(Level level, String leaf) {
            m_level = level;
            m_pose2dLogger = pose2dLogger(level, append(leaf, "pose"));
            m_rotation2dLogger = rotation2dLogger(level, append(leaf, "course"));
        }

        public void log(Supplier<Pose2dWithMotion> vals) {
            if (!allow(m_level))
                return;
            Pose2dWithMotion val = vals.get();
            m_pose2dLogger.log(val::getPose);
            Optional<Rotation2d> course = val.getCourse();
            if (course.isPresent()) {
                m_rotation2dLogger.log(course::get);
            }
        }
    }

    public Pose2dWithMotionLogger pose2dWithMotionLogger(Level level, String leaf) {
        return new Pose2dWithMotionLogger(level, leaf);
    }

    public class Twist2dLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_dxLogger;
        private final DoubleSupplierLogger m_dyLogger;
        private final DoubleSupplierLogger m_dthetaLogger;

        Twist2dLogger(Level level, String leaf) {
            m_level = level;
            m_dxLogger = doubleLogger(level, append(leaf, "dx"));
            m_dyLogger = doubleLogger(level, append(leaf, "dy"));
            m_dthetaLogger = doubleLogger(level, append(leaf, "dtheta"));
        }

        public void log(Supplier<Twist2d> vals) {
            if (!allow(m_level))
                return;
            Twist2d val = vals.get();
            m_dxLogger.log(() -> val.dx);
            m_dyLogger.log(() -> val.dy);
            m_dthetaLogger.log(() -> val.dtheta);
        }
    }

    public Twist2dLogger twist2dLogger(Level level, String leaf) {
        return new Twist2dLogger(level, leaf);
    }

    public class ChassisSpeedsLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_vxLogger;
        private final DoubleSupplierLogger m_vyLogger;
        private final DoubleSupplierLogger m_omegaLogger;

        ChassisSpeedsLogger(Level level, String leaf) {
            m_level = level;
            m_vxLogger = doubleLogger(level, append(leaf, "vx m_s"));
            m_vyLogger = doubleLogger(level, append(leaf, "vy m_s"));
            m_omegaLogger = doubleLogger(level, append(leaf, "omega rad_s"));
        }

        public void log(Supplier<ChassisSpeeds> vals) {
            if (!allow(m_level))
                return;
            ChassisSpeeds val = vals.get();
            m_vxLogger.log(() -> val.vxMetersPerSecond);
            m_vyLogger.log(() -> val.vyMetersPerSecond);
            m_omegaLogger.log(() -> val.omegaRadiansPerSecond);
        }
    }

    public ChassisSpeedsLogger chassisSpeedsLogger(Level level, String leaf) {
        return new ChassisSpeedsLogger(level, leaf);
    }

    public class FieldRelativeVelocityLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_xLogger;
        private final DoubleSupplierLogger m_yLogger;
        private final DoubleSupplierLogger m_thetaLogger;

        FieldRelativeVelocityLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, append(leaf, "x m_s"));
            m_yLogger = doubleLogger(level, append(leaf, "y m_s"));
            m_thetaLogger = doubleLogger(level, append(leaf, "theta rad_s"));
        }

        public void log(Supplier<FieldRelativeVelocity> vals) {
            if (!allow(m_level))
                return;
            FieldRelativeVelocity val = vals.get();
            m_xLogger.log(val::x);
            m_yLogger.log(val::y);
            m_thetaLogger.log(val::theta);
        }
    }

    public FieldRelativeVelocityLogger fieldRelativeVelocityLogger(Level level, String leaf) {
        return new FieldRelativeVelocityLogger(level, leaf);
    }

    public class FieldRelativeAccelerationLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_xLogger;
        private final DoubleSupplierLogger m_yLogger;
        private final DoubleSupplierLogger m_thetaLogger;

        FieldRelativeAccelerationLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, append(leaf, "x m_s_s"));
            m_yLogger = doubleLogger(level, append(leaf, "y m_s_s"));
            m_thetaLogger = doubleLogger(level, append(leaf, "theta rad_s_s"));
        }

        public void log(Supplier<FieldRelativeAcceleration> vals) {
            if (!allow(m_level))
                return;
            FieldRelativeAcceleration val = vals.get();
            m_xLogger.log(val::x);
            m_yLogger.log(val::y);
            m_thetaLogger.log(val::theta);
        }
    }

    public FieldRelativeAccelerationLogger fieldRelativeAccelerationLogger(Level level, String leaf) {
        return new FieldRelativeAccelerationLogger(level, leaf);
    }

    public class State100Logger {
        private final Level m_level;
        private final DoubleSupplierLogger m_xLogger;
        private final DoubleSupplierLogger m_vLogger;
        private final DoubleSupplierLogger m_aLogger;

        State100Logger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, append(leaf, "x"));
            m_vLogger = doubleLogger(level, append(leaf, "v"));
            m_aLogger = doubleLogger(level, append(leaf, "a"));
        }

        public void log(Supplier<State100> vals) {
            if (!allow(m_level))
                return;
            State100 val = vals.get();
            m_xLogger.log(val::x);
            m_vLogger.log(val::v);
            m_aLogger.log(val::a);
        }
    }

    public State100Logger state100Logger(Level level, String leaf) {
        return new State100Logger(level, leaf);
    }

    public class SwerveStateLogger {
        private final Level m_level;
        private final State100Logger m_xLogger;
        private final State100Logger m_yLogger;
        private final State100Logger m_thetaLogger;

        SwerveStateLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = state100Logger(level, append(leaf, "x"));
            m_yLogger = state100Logger(level, append(leaf, "y"));
            m_thetaLogger = state100Logger(level, append(leaf, "theta"));
        }

        public void log(Supplier<SwerveState> vals) {
            if (!allow(m_level))
                return;
            SwerveState val = vals.get();
            m_xLogger.log(val::x);
            m_yLogger.log(val::y);
            m_thetaLogger.log(val::theta);
        }
    }

    public SwerveStateLogger swerveStateLogger(Level level, String leaf, Supplier<SwerveState> vals) {
        return new SwerveStateLogger(level, leaf);
    }

    public class SwerveModulePosition100Logger {
        private final Level m_level;
        private final DoubleSupplierLogger m_distanceLogger;
        private final Rotation2dLogger m_rotation2dLogger;

        SwerveModulePosition100Logger(Level level, String leaf) {
            m_level = level;
            m_distanceLogger = doubleLogger(level, append(leaf, "distance"));
            m_rotation2dLogger = rotation2dLogger(level, append(leaf, "angle"));
        }

        public void log(Supplier<SwerveModulePosition100> vals) {
            if (!allow(m_level))
                return;
            SwerveModulePosition100 val = vals.get();
            m_distanceLogger.log(() -> val.distanceMeters);
            if (val.angle.isPresent()) {
                m_rotation2dLogger.log(val.angle::get);
            }
        }
    }

    public SwerveModulePosition100Logger logSwerveModulePosition100(Level level, String leaf) {
        return new SwerveModulePosition100Logger(level, leaf);
    }

    public class ArmAnglesLogger {
        private final Level m_level;
        private final DoubleSupplierLogger m_th1Logger;
        private final DoubleSupplierLogger m_th2Logger;

        ArmAnglesLogger(Level level, String leaf) {
            m_level = level;
            m_th1Logger = doubleLogger(level, append(leaf, "th1"));
            m_th2Logger = doubleLogger(level, append(leaf, "th2"));
        }

        public void log(Supplier<ArmAngles> vals) {
            if (!allow(m_level))
                return;
            ArmAngles val = vals.get();
            m_th1Logger.log(() -> val.th1);
            m_th2Logger.log(() -> val.th2);
        }
    }

    public ArmAnglesLogger armAnglesLogger(Level level, String leaf) {
        return new ArmAnglesLogger(level, leaf);
    }

    public class StateLogger {
        private final Level m_level;
        private final Pose2dLogger m_poseLogger;
        private final DoubleSupplierLogger m_curvatureLogger;
        private final DoubleSupplierLogger m_velocityLogger;
        private final DoubleSupplierLogger m_accelLogger;

        StateLogger(Level level, String leaf) {
            m_level = level;
            m_poseLogger = pose2dLogger(level, append(leaf, "pose"));
            m_curvatureLogger = doubleLogger(level, append(leaf, "curvature"));
            m_velocityLogger = doubleLogger(level, append(leaf, "velocity"));
            m_accelLogger = doubleLogger(level, append(leaf, "accel"));
        }

        public void log(Supplier<State> vals) {
            if (!allow(m_level))
                return;
            State val = vals.get();
            m_poseLogger.log(() -> val.poseMeters);
            m_curvatureLogger.log(() -> val.curvatureRadPerMeter);
            m_velocityLogger.log(() -> val.velocityMetersPerSecond);
            m_accelLogger.log(() -> val.accelerationMetersPerSecondSq);
        }
    }

    public StateLogger logState(Level level, String leaf, Supplier<State> vals) {
        return new StateLogger(level, leaf);
    }

    public class Blip24Logger {
        private final Level m_level;
        private final IntSupplierLogger m_idLogger;
        private final Transform3dLogger m_transformLogger;

        Blip24Logger(Level level, String leaf) {
            m_level = level;
            m_idLogger = intLogger(level, append(leaf, "id"));
            m_transformLogger = transform3dLogger(level, append(leaf, "transform"));
        }

        public void log(Supplier<Blip24> vals) {
            if (!allow(m_level))
                return;
            Blip24 val = vals.get();
            m_idLogger.log(val::getId);
            m_transformLogger.log(val::getPose);
        }
    }

    public Blip24Logger logBlip24(Level level, String leaf, Supplier<Blip24> vals) {
        return new Blip24Logger(level, leaf);
    }
}

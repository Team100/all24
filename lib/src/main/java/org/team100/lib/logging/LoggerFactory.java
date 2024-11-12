package org.team100.lib.logging;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.localization.Blip24;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.drivetrain.SwerveControl;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
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

/**
 * This is the logger factory class.
 * 
 * The use pattern is:
 * 
 * * pass this through the tree of constructors
 * * each class makes a "child" -- the logger tree follows instantiation
 * * use the child to create logger instances as members
 * * use the member logger instances at whatever site you want
 * 
 * In general you shouldn't keep references to this factory; let the top level
 * container keep the root reference.
 * 
 * Don't use slashes in names, it confuses Glass.
 */
public class LoggerFactory {
    private final Supplier<Level> m_level;
    private final String m_root;
    private final PrimitiveLogger m_pLogger;

    public LoggerFactory(
            Supplier<Level> level,
            String root,
            PrimitiveLogger primitiveLogger) {
        if (root.startsWith("/"))
            throw new IllegalArgumentException("don't lead with a slash");
        m_level = level;
        m_root = root;
        m_pLogger = primitiveLogger;
    }

    /**
     * Use this to create a child logger that describes the purpose of the
     * subordinate thing, e.g. the "left" thing or the "right" thing.
     * 
     * Each child level is separated by slashes, to make a tree in glass.
     */
    public LoggerFactory child(String stem) {
        return new LoggerFactory(m_level, m_root + "/" + stem, m_pLogger);
    }

    /**
     * Use this to create a child logger that describes the type of the subordinate
     * thing, using the stable glass name (i.e. interface names, not implementation
     * names, where possible).
     */
    public LoggerFactory child(Glassy obj) {
        return child(obj.getGlassName());
    }

    private boolean allow(Level level) {
        Level allowed = m_level.get();
        if (allowed == Level.COMP && level == Level.COMP) {
            // comp mode allows COMP level regardless of enablement.
            return true;
        }
        return allowed.admit(level);
    }

    /** @return root/stem */
    private String root(String stem) {
        return m_root + "/" + stem;
    }

    /** @return a/b */
    private String join(String a, String b) {
        return a + "/" + b;
    }

    /////////////////////////////////////////////////////
    //
    // logger inner classes
    //

    public class BooleanLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveBooleanLogger m_primitiveLogger;

        BooleanLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.booleanLogger(root(leaf));
        }

        public void log(BooleanSupplier vals) {
            if (!allow(m_level))
                return;
            boolean val = vals.getAsBoolean();
            m_primitiveLogger.log(val);
        }
    }

    public BooleanLogger booleanLogger(Level level, String leaf) {
        return new BooleanLogger(level, leaf);
    }

    public class DoubleLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveDoubleLogger m_primitiveLogger;

        DoubleLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.doubleLogger(root(leaf));
        }

        public void log(DoubleSupplier vals) {
            if (!allow(m_level))
                return;
            double val = vals.getAsDouble();
            m_primitiveLogger.log(val);
        }

        public void log(Supplier<Double> vals) {
            if (!allow(m_level))
                return;
            double val = vals.get();
            m_primitiveLogger.log(val);
        }
    }

    public DoubleLogger doubleLogger(Level level, String leaf) {
        return new DoubleLogger(level, leaf);
    }

    public class IntLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveIntLogger m_primitiveLogger;

        IntLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.intLogger(root(leaf));
        }

        public void log(IntSupplier vals) {
            if (!allow(m_level))
                return;
            int val = vals.getAsInt();
            m_primitiveLogger.log(val);
        }
    }

    public IntLogger intLogger(Level level, String leaf) {
        return new IntLogger(level, leaf);
    }

    public class DoubleArrayLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveDoubleArrayLogger m_primitiveLogger;

        DoubleArrayLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.doubleArrayLogger(root(leaf));
        }

        public void log(Supplier<double[]> vals) {
            if (!allow(m_level))
                return;
            double[] val = vals.get();
            m_primitiveLogger.log(val);
        }
    }

    public DoubleArrayLogger doubleArrayLogger(Level level, String leaf) {
        return new DoubleArrayLogger(level, leaf);
    }

    public class LongLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveLongLogger m_primitiveLogger;

        LongLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.longLogger(root(leaf));
        }

        public void log(LongSupplier vals) {
            if (!allow(m_level))
                return;
            long val = vals.getAsLong();
            m_primitiveLogger.log(val);
        }
    }

    public LongLogger longLogger(Level level, String leaf) {
        return new LongLogger(level, leaf);
    }

    public class StringLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveStringLogger m_primitiveLogger;

        StringLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.stringLogger(root(leaf));
        }

        public void log(Supplier<String> vals) {
            if (!allow(m_level))
                return;
            String val = vals.get();
            m_primitiveLogger.log(val);
        }
    }

    public StringLogger stringLogger(Level level, String leaf) {
        return new StringLogger(level, leaf);
    }

    public class OptionalDoubleLogger {
        private final Level m_level;
        private final PrimitiveLogger.PrimitiveDoubleLogger m_primitiveLogger;

        OptionalDoubleLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.doubleLogger(root(leaf));
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
        private final PrimitiveLogger.PrimitiveStringLogger m_primitiveLogger;

        EnumLogger(Level level, String leaf) {
            m_level = level;
            m_primitiveLogger = m_pLogger.stringLogger(root(leaf));
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
            m_translation2dLogger = translation2dLogger(level, join(leaf, "translation"));
            m_rotation2dLogger = rotation2dLogger(level, join(leaf, "rotation"));
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
            m_translation3dLogger = translation3dLogger(level, join(leaf, "translation"));
            m_rotation3dLogger = rotation3dLogger(level, join(leaf, "rotation"));
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
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_yLogger;
        private final DoubleLogger m_zLogger;

        Translation3dLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x"));
            m_yLogger = doubleLogger(level, join(leaf, "y"));
            m_zLogger = doubleLogger(level, join(leaf, "z"));
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
        private final DoubleLogger m_rollLogger;
        private final DoubleLogger m_pitchLogger;
        private final DoubleLogger m_yawLogger;

        Rotation3dLogger(Level level, String leaf) {
            m_level = level;
            m_rollLogger = doubleLogger(level, join(leaf, "roll"));
            m_pitchLogger = doubleLogger(level, join(leaf, "pitch"));
            m_yawLogger = doubleLogger(level, join(leaf, "yaw"));
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
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_yLogger;

        Translation2dLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x"));
            m_yLogger = doubleLogger(level, join(leaf, "y"));
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
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_yLogger;

        Vector2dLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x"));
            m_yLogger = doubleLogger(level, join(leaf, "y"));
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
        private final DoubleLogger m_radLogger;

        Rotation2dLogger(Level level, String leaf) {
            m_level = level;
            m_radLogger = doubleLogger(level, join(leaf, "rad"));
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
            m_timedPoseLogger = timedPoseLogger(level, join(leaf, "state"));
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
        private final DoubleLogger m_timeLogger;
        private final DoubleLogger m_velocityLogger;
        private final DoubleLogger m_accelLogger;

        TimedPoseLogger(Level level, String leaf) {
            m_level = level;
            m_pose2dWithMotionLogger = pose2dWithMotionLogger(level, join(leaf, "posestate"));
            m_timeLogger = doubleLogger(level, join(leaf, "time"));
            m_velocityLogger = doubleLogger(level, join(leaf, "velocity"));
            m_accelLogger = doubleLogger(level, join(leaf, "accel"));
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
            m_pose2dLogger = pose2dLogger(level, join(leaf, "pose"));
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
            m_pose2dLogger = pose2dLogger(level, join(leaf, "pose"));
            m_rotation2dLogger = rotation2dLogger(level, join(leaf, "course"));
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
        private final DoubleLogger m_dxLogger;
        private final DoubleLogger m_dyLogger;
        private final DoubleLogger m_dthetaLogger;

        Twist2dLogger(Level level, String leaf) {
            m_level = level;
            m_dxLogger = doubleLogger(level, join(leaf, "dx"));
            m_dyLogger = doubleLogger(level, join(leaf, "dy"));
            m_dthetaLogger = doubleLogger(level, join(leaf, "dtheta"));
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
        private final DoubleLogger m_vxLogger;
        private final DoubleLogger m_vyLogger;
        private final DoubleLogger m_omegaLogger;

        ChassisSpeedsLogger(Level level, String leaf) {
            m_level = level;
            m_vxLogger = doubleLogger(level, join(leaf, "vx m_s"));
            m_vyLogger = doubleLogger(level, join(leaf, "vy m_s"));
            m_omegaLogger = doubleLogger(level, join(leaf, "omega rad_s"));
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
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_yLogger;
        private final DoubleLogger m_thetaLogger;

        FieldRelativeVelocityLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x m_s"));
            m_yLogger = doubleLogger(level, join(leaf, "y m_s"));
            m_thetaLogger = doubleLogger(level, join(leaf, "theta rad_s"));
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
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_yLogger;
        private final DoubleLogger m_thetaLogger;

        FieldRelativeAccelerationLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x m_s_s"));
            m_yLogger = doubleLogger(level, join(leaf, "y m_s_s"));
            m_thetaLogger = doubleLogger(level, join(leaf, "theta rad_s_s"));
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

    public class Model100Logger {
        private final Level m_level;
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_vLogger;

        Model100Logger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x"));
            m_vLogger = doubleLogger(level, join(leaf, "v"));
        }

        public void log(Supplier<Model100> vals) {
            if (!allow(m_level))
                return;
            Model100 val = vals.get();
            m_xLogger.log(val::x);
            m_vLogger.log(val::v);
        }
    }

    public class Control100Logger {
        private final Level m_level;
        private final DoubleLogger m_xLogger;
        private final DoubleLogger m_vLogger;
        private final DoubleLogger m_aLogger;

        Control100Logger(Level level, String leaf) {
            m_level = level;
            m_xLogger = doubleLogger(level, join(leaf, "x"));
            m_vLogger = doubleLogger(level, join(leaf, "v"));
            m_aLogger = doubleLogger(level, join(leaf, "a"));
        }

        public void log(Supplier<Control100> vals) {
            if (!allow(m_level))
                return;
            Control100 val = vals.get();
            m_xLogger.log(val::x);
            m_vLogger.log(val::v);
            m_aLogger.log(val::a);
        }
    }

    public Control100Logger control100Logger(Level level, String leaf) {
        return new Control100Logger(level, leaf);
    }

    public class SwerveControlLogger {
        private final Level m_level;
        private final Control100Logger m_xLogger;
        private final Control100Logger m_yLogger;
        private final Control100Logger m_thetaLogger;

        SwerveControlLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = control100Logger(level, join(leaf, "x"));
            m_yLogger = control100Logger(level, join(leaf, "y"));
            m_thetaLogger = control100Logger(level, join(leaf, "theta"));
        }

        public void log(Supplier<SwerveControl> vals) {
            if (!allow(m_level))
                return;
            SwerveControl val = vals.get();
            m_xLogger.log(val::x);
            m_yLogger.log(val::y);
            m_thetaLogger.log(val::theta);
        }
    }

    public SwerveControlLogger swerveControlLogger(Level level, String leaf) {
        return new SwerveControlLogger(level, leaf);
    }

    public Model100Logger model100Logger(Level level, String leaf) {
        return new Model100Logger(level, leaf);
    }

    public class SwerveModelLogger {
        private final Level m_level;
        private final Model100Logger m_xLogger;
        private final Model100Logger m_yLogger;
        private final Model100Logger m_thetaLogger;

        SwerveModelLogger(Level level, String leaf) {
            m_level = level;
            m_xLogger = model100Logger(level, join(leaf, "x"));
            m_yLogger = model100Logger(level, join(leaf, "y"));
            m_thetaLogger = model100Logger(level, join(leaf, "theta"));
        }

        public void log(Supplier<SwerveModel> vals) {
            if (!allow(m_level))
                return;
            SwerveModel val = vals.get();
            m_xLogger.log(val::x);
            m_yLogger.log(val::y);
            m_thetaLogger.log(val::theta);
        }
    }

    public SwerveModelLogger swerveModelLogger(Level level, String leaf) {
        return new SwerveModelLogger(level, leaf);
    }

    public class SwerveModulePosition100Logger {
        private final Level m_level;
        private final DoubleLogger m_distanceLogger;
        private final Rotation2dLogger m_rotation2dLogger;

        SwerveModulePosition100Logger(Level level, String leaf) {
            m_level = level;
            m_distanceLogger = doubleLogger(level, join(leaf, "distance"));
            m_rotation2dLogger = rotation2dLogger(level, join(leaf, "angle"));
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

    public SwerveModulePosition100Logger swerveModulePosition100Logger(Level level, String leaf) {
        return new SwerveModulePosition100Logger(level, leaf);
    }

    public class ArmAnglesLogger {
        private final Level m_level;
        private final DoubleLogger m_th1Logger;
        private final DoubleLogger m_th2Logger;

        ArmAnglesLogger(Level level, String leaf) {
            m_level = level;
            m_th1Logger = doubleLogger(level, join(leaf, "th1"));
            m_th2Logger = doubleLogger(level, join(leaf, "th2"));
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
        private final DoubleLogger m_curvatureLogger;
        private final DoubleLogger m_velocityLogger;
        private final DoubleLogger m_accelLogger;

        StateLogger(Level level, String leaf) {
            m_level = level;
            m_poseLogger = pose2dLogger(level, join(leaf, "pose"));
            m_curvatureLogger = doubleLogger(level, join(leaf, "curvature"));
            m_velocityLogger = doubleLogger(level, join(leaf, "velocity"));
            m_accelLogger = doubleLogger(level, join(leaf, "accel"));
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

    public StateLogger logState(Level level, String leaf) {
        return new StateLogger(level, leaf);
    }

    public class Blip24Logger {
        private final Level m_level;
        private final IntLogger m_idLogger;
        private final Transform3dLogger m_transformLogger;

        Blip24Logger(Level level, String leaf) {
            m_level = level;
            m_idLogger = intLogger(level, join(leaf, "id"));
            m_transformLogger = transform3dLogger(level, join(leaf, "transform"));
        }

        public void log(Supplier<Blip24> vals) {
            if (!allow(m_level))
                return;
            Blip24 val = vals.get();
            m_idLogger.log(val::getId);
            m_transformLogger.log(val::getPose);
        }
    }

    public Blip24Logger logBlip24(Level level, String leaf) {
        return new Blip24Logger(level, leaf);
    }
}

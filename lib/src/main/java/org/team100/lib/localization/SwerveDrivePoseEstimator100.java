package org.team100.lib.localization;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Map.Entry;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.persistent_parameter.ParameterFactory;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Names;
import org.team100.lib.util.SlipperyTireUtil;
import org.team100.lib.util.Tire;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Collapses WPI SwerveDrivePoseEstimator and PoseEstimator.
 *
 * call update() periodically.
 *
 * call addVisionMeasurement} asynchronously.
 */
public class SwerveDrivePoseEstimator100 implements PoseEstimator100, Glassy {
    private static final double kBufferDuration = 1.5;
    // look back a little to get a pose for velocity estimation
    private static final double velocityDtS = 0.02;

    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final int m_numModules;
    private final SwerveDriveKinematics100 m_kinematics;
    private final Matrix<N3, N1> m_q;
    private final Matrix<N3, N3> m_visionK;
    private final TimeInterpolatableBuffer100<InterpolationRecord> m_poseBuffer;

    // for now, default tire.

    public ParameterFactory f = new ParameterFactory(new HashMap<>());

    Tire tire = new Tire(f);
    SlipperyTireUtil u = new SlipperyTireUtil(tire);
    /**
     * maintained in resetPosition().
     */
    Rotation2d m_gyroOffset;

    /**
     *
     * @param kinematics               A correctly-configured kinematics object for
     *                                 your drivetrain.
     * @param gyroAngle                The current gyro angle.
     * @param modulePositions          The current distance and rotation
     *                                 measurements of the swerve modules.
     * @param initialPoseMeters        The starting pose estimate.
     * @param stateStdDevs             Standard deviations of the pose estimate (x
     *                                 position in meters, y position
     *                                 in meters, and heading in radians). Increase
     *                                 these numbers to trust your state estimate
     *                                 less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public SwerveDrivePoseEstimator100(
            SwerveDriveKinematics100 kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        m_name = Names.name(this);

        m_numModules = modulePositions.length;
        m_kinematics = kinematics;
        m_q = new Matrix<>(Nat.N3(), Nat.N1());
        m_visionK = new Matrix<>(Nat.N3(), Nat.N3());
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                kBufferDuration,
                timestampSeconds,
                new InterpolationRecord(
                        m_kinematics,
                        initialPoseMeters,
                        gyroAngle,
                        new SwerveDriveWheelPositions(modulePositions)));
        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        setStdDevs(stateStdDevs, visionMeasurementStdDevs);
    }

    @Override
    public void setStdDevs(
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                        row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
            }
        }
    }

    /**
     * Gets the estimated robot pose.
     * 
     * This should really only be used by other threads, so the order of
     * update and reading doesn't matter.
     */
    public Pose2d getEstimatedPosition() {
        return m_poseBuffer.lastEntry().getValue().m_poseMeters;
    }

    @Override
    public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
        InterpolationRecord sample = m_poseBuffer.get(timestampSeconds);
        return Optional.of(sample.m_poseMeters.getRotation());
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: If this measurement is old enough to be outside the pose buffer's
        // timespan, skip.

        if (m_poseBuffer.lastKey() - kBufferDuration > timestampSeconds) {
            return;
        }

        // Step 1: Get the pose odometry measured at the moment the vision measurement
        // was made.
        InterpolationRecord sample = m_poseBuffer.get(timestampSeconds);

        // Step 2: Measure the twist between the odometry pose and the vision pose.
        Twist2d twist = sample.m_poseMeters.log(visionRobotPoseMeters);

        // Step 3: We should not trust the twist entirely, so instead we scale this
        // twist by a Kalman gain matrix representing how much we trust vision
        // measurements compared to our current pose.
        Matrix<N3, N1> k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

        // Step 4: Convert back to Twist2d.
        Twist2d scaledTwist = new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

        Pose2d newPose = sample.m_poseMeters.exp(scaledTwist);

        // Step 5: Adjust the gyro offset so that the adjusted pose is consistent with
        // the unadjusted gyro angle
        // this should have no effect if you disregard vision angle input

        m_gyroOffset = newPose.getRotation().minus(sample.m_gyroAngle);
        t.log(Level.TRACE, m_name, "GYRO OFFSET", m_gyroOffset);

        // Step 6: Record the current pose to allow multiple measurements from the same
        // timestamp
        m_poseBuffer.put(
                timestampSeconds,
                new InterpolationRecord(
                        m_kinematics,
                        newPose,
                        sample.m_gyroAngle,
                        sample.m_wheelPositions));

        // Step 7: Replay odometry inputs between sample time and latest recorded sample
        // to update the pose buffer and correct odometry.
        // note exclusive tailmap, don't need to reprocess the entry we just put there.
        for (Map.Entry<Double, InterpolationRecord> entry : m_poseBuffer.tailMap(timestampSeconds, false).entrySet()) {
            double entryTimestampS = entry.getKey();
            Rotation2d entryGyroAngle = entry.getValue().m_gyroAngle;
            SwerveDriveWheelPositions wheelPositions = entry.getValue().m_wheelPositions;
            update(entryTimestampS, entryGyroAngle, wheelPositions);
        }
    }

    public void resetPosition(
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions,
            Pose2d pose,
            double timestampSeconds) {

        checkLength(modulePositions);

        m_gyroOffset = pose.getRotation().minus(gyroAngle);

        // empty the buffer and add the current pose
        m_poseBuffer.reset(
                timestampSeconds,
                new InterpolationRecord(m_kinematics, pose, gyroAngle, modulePositions.copy()));

        t.log(Level.TRACE, m_name, "GYRO OFFSET", m_gyroOffset);
    }

    void resetOdometry(
            Rotation2d gyroAngle,
            Pose2d pose) {
        m_gyroOffset = pose.getRotation().minus(gyroAngle);
        t.log(Level.TRACE, m_name, "GYRO OFFSET", m_gyroOffset);
    }

    /**
     * Allow vision and stdev changes in one fn.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        setStdDevs(stateStdDevs, visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information and
     * returns
     * the pose estimate for the given time.
     * 
     * This should be called periodically.
     *
     * @param currentTimeS   Time at which this method was called, in seconds.
     * @param gyroAngle      Current gyroscope angle.
     * @param wheelPositions Current distance measurements and rotations of
     *                       the swerve modules.
     * @return The estimated pose of the robot at the given time.
     */
    public Pose2d update(
            double currentTimeS,
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions wheelPositions) {
        checkLength(wheelPositions);

        List<Entry<Double, InterpolationRecord>> consistentPair = m_poseBuffer.consistentPair(
                currentTimeS, velocityDtS);

        if (consistentPair.isEmpty()) {
            // we're at the beginning. there's nothing to apply the wheel position delta to.
            // the buffer is never empty, so there's always a ceiling.
            return m_poseBuffer.ceilingEntry(currentTimeS).getValue().m_poseMeters;
        }

        // the entry right before this one, the basis for integration.
        Entry<Double, InterpolationRecord> lowerEntry = consistentPair.get(0);

        double t1 = currentTimeS - lowerEntry.getKey();
        t.log(Level.DEBUG, m_name, "t1", t1);
        InterpolationRecord value = lowerEntry.getValue();
        Pose2d previousPose = value.m_poseMeters;

        SwerveModulePosition[] modulePositionDelta = DriveUtil.modulePositionDelta(
                value.m_wheelPositions,
                wheelPositions);

        if (consistentPair.size() > 1) {
            // get an earlier pose in order to adjust the corner velocities
            Map.Entry<Double, InterpolationRecord> earlierEntry = consistentPair.get(1);

            double t0 = lowerEntry.getKey() - earlierEntry.getKey();
            t.log(Level.DEBUG, m_name, "t0", t0);
            Pose2d earlierPose = earlierEntry.getValue().m_poseMeters;
            Vector2d[] corners = SlipperyTireUtil.cornerDeltas(
                    m_kinematics,
                    earlierPose,
                    previousPose);
            t.log(Level.DEBUG, m_name, "delta0", modulePositionDelta[0]);
            modulePositionDelta = u.adjust(corners, t0, modulePositionDelta, t1);
            t.log(Level.DEBUG, m_name, "delta1", modulePositionDelta[0]);
        }

        Twist2d twist = m_kinematics.toTwist2d(modulePositionDelta);

        // replace the twist dtheta with one derived from the current pose
        // pose angle based on the gyro (which is more accurate)

        Rotation2d angle = gyroAngle.plus(m_gyroOffset);
        twist.dtheta = angle.minus(previousPose.getRotation()).getRadians();

        Pose2d newPose = new Pose2d(previousPose.exp(twist).getTranslation(), angle);

        t.log(Level.TRACE, m_name, "posex", newPose.getX());

        m_poseBuffer.put(
                currentTimeS,
                new InterpolationRecord(m_kinematics, newPose, gyroAngle, wheelPositions.copy()));

        Transform2d deltaTransform = newPose.minus(previousPose).div(t1);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(
                deltaTransform.getX(),
                deltaTransform.getY(),
                deltaTransform.getRotation().getRadians());

        // TODO finish this
        SwerveState state = new SwerveState(newPose, velocity, new FieldRelativeAcceleration(0, 0, 0));

        // this produces a *lot* of output
        // dump();
        return newPose;
    }

    @Override
    public String getGlassName() {
        return "SwerveDrivePoseEstimator100";
    }

    ///////////////////////////////////////

    private void checkLength(SwerveDriveWheelPositions modulePositions) {
        int ct = modulePositions.positions.length;
        if (ct != m_numModules) {
            throw new IllegalArgumentException("Wrong module count: " + ct);
        }
    }
}

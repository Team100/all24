package org.team100.lib.copies;

import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.Optional;
import java.util.Map.Entry;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.localization.PoseEstimator100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Names;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
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

    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final int m_numModules;
    private final SwerveDriveKinematics100 m_kinematics;
    private final Matrix<N3, N1> m_q;
    private final Matrix<N3, N3> m_visionK;
    private final TimeInterpolatableBuffer100<InterpolationRecord> m_poseBuffer;

    /**
     * "current" pose, maintained in update() and resetPosition().
     */
    private Pose2d m_poseMeters;

    /**
     * maintained in resetPosition().
     */
    private Rotation2d m_gyroOffset;

    /**
     * maintained in update() and resetPosition()
     */
    private SwerveDriveWheelPositions m_previousWheelPositions;

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
        m_poseBuffer = TimeInterpolatableBuffer100.createBuffer(kBufferDuration);

        m_poseMeters = initialPoseMeters;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousWheelPositions = new SwerveDriveWheelPositions(modulePositions).copy();

        setStdDevs(stateStdDevs, visionMeasurementStdDevs);

        // plant the starting point in the buffer
        // System.out.println("add sample at time " + timestampSeconds);
        m_poseBuffer.addSample(
                timestampSeconds,
                new InterpolationRecord(
                        initialPoseMeters,
                        gyroAngle,
                        m_previousWheelPositions.copy()));
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
        // return m_poseMeters;
        return m_poseBuffer.lastEntry().getValue().poseMeters;
    }

    public void dump() {
        for (Entry<Double, InterpolationRecord> e : m_poseBuffer.tailMap(0, true).entrySet()) {
            System.out.printf("%f %f\n", e.getKey(), e.getValue().poseMeters.getX());
        }
    }

    @Override
    public Optional<Rotation2d> getSampledRotation(double timestampSeconds) {
        Optional<InterpolationRecord> sample = m_poseBuffer.getSample(timestampSeconds);
        if (sample.isEmpty())
            return Optional.empty();
        // return Optional.of(sample.get().gyroAngle);
        return Optional.of(sample.get().poseMeters.getRotation());
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds) {
        // Step 0: If this measurement is old enough to be outside the pose buffer's
        // timespan, skip.
        try {
            if (m_poseBuffer.lastKey() - kBufferDuration > timestampSeconds) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        // Step 1: Get the pose odometry measured at the moment the vision measurement
        // was made.
        Optional<InterpolationRecord> optionalSample = m_poseBuffer.getSample(timestampSeconds);

        if (optionalSample.isEmpty()) {
            return;
        }
        InterpolationRecord sample = optionalSample.get();

        // Step 2: Measure the twist between the odometry pose and the vision pose.
        Twist2d twist = sample.poseMeters.log(visionRobotPoseMeters);

        // Step 3: We should not trust the twist entirely, so instead we scale this
        // twist by a Kalman gain matrix representing how much we trust vision
        // measurements compared to our current pose.
        Matrix<N3, N1> k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

        // Step 4: Convert back to Twist2d.
        Twist2d scaledTwist = new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

        Pose2d newPose = sample.poseMeters.exp(scaledTwist);

        // Step 5: Reset Odometry to state at sample with vision adjustment.
        // it's super weird that this resets the odometry briefly as part of fixing the
        // history.
        // because we do these two things with two different threads.
        // TODO: blarg
        resetOdometry(
                sample.gyroAngle,
                sample.wheelPositions,
                newPose);

        // Step 6: Record the current pose to allow multiple measurements from the same
        // timestamp

        // System.out.println("add sample at time " + timestampSeconds);
        m_poseBuffer.addSample(
                timestampSeconds,
                new InterpolationRecord(
                        newPose,
                        sample.gyroAngle,
                        sample.wheelPositions));

        // Step 7: Replay odometry inputs between sample time and latest recorded sample
        // to update the pose buffer and correct odometry.
        // note exclusive tailmap, don't need to reprocess the entry we just put there.
        for (Map.Entry<Double, InterpolationRecord> entry : m_poseBuffer.tailMap(timestampSeconds, false).entrySet()) {
            double entryTimestampS = entry.getKey();
            Rotation2d entryGyroAngle = entry.getValue().gyroAngle;
            SwerveDriveWheelPositions wheelPositions = entry.getValue().wheelPositions;
            // System.out.println("update entry for time " + entryTimestampS);
            update(entryTimestampS, entryGyroAngle, wheelPositions);
        }
    }

    public void resetPosition(
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions,
            Pose2d pose,
            double timestampSeconds) {
        resetOdometry(gyroAngle, modulePositions, pose);
        m_poseBuffer.clear();
        // keep the current pose in the buffer
        // System.out.println("add sample at time " + timestampSeconds);
        m_poseBuffer.addSample(
                timestampSeconds,
                new InterpolationRecord(pose, gyroAngle, modulePositions.copy()));
    }

    void resetOdometry(
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions,
            Pose2d pose) {
        checkLength(modulePositions);
        // TODO: remove this
        m_poseMeters = pose;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousWheelPositions = modulePositions.copy();
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
     * Updates the pose estimator with wheel encoder and gyro information.
     * 
     * This should be called periodically.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     *                           must be monotonic since the odometry expects that.
     * @param gyroAngle          The current gyroscope angle.
     * @param wheelPositions     The current distance measurements and rotations of
     *                           the swerve modules.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d update(
            double currentTimeSeconds,
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions wheelPositions) {
        checkLength(wheelPositions);

        // the last entry should be the same as m_poseMeters
        // Entry<Double, InterpolationRecord> lastEntry = m_poseBuffer.lastEntry();
        // if we're replaying the past, we want the entry right before this one.
        Entry<Double, InterpolationRecord> lowerEntry = m_poseBuffer.lowerEntry(currentTimeSeconds);
       
        if (lowerEntry == null) {
            // we're at the beginning, or there haven't been any updates in a long
            // time so the cleaner has removed them all.
            // there's nothing to apply the wheel position delta to, so just
            // return something?
            return m_poseBuffer.ceilingEntry(currentTimeSeconds).getValue().poseMeters;
        }
        // these should be the same
        if (Math.abs(lowerEntry.getValue().poseMeters.getX() - m_poseMeters.getX()) > 0.0000001)
            throw new IllegalArgumentException(
                    "blarg " + lowerEntry.getValue().poseMeters.getX() + " " + m_poseMeters.getX());
        // System.out.printf("%f %f\n", m_poseMeters.getX(),
        // lastEntry.getValue().poseMeters.getX());

        // TODO: this should take tires into account!
        SwerveModulePosition[] modulePositionDelta = DriveUtil.modulePositionDelta(
                m_previousWheelPositions,
                wheelPositions);
        Twist2d twist = m_kinematics.toTwist2d(modulePositionDelta);

        // replace the twist dtheta with one derived from the current pose
        // pose angle based on the gyro (which is more accurate)
        Rotation2d angle = gyroAngle.plus(m_gyroOffset);
        twist.dtheta = angle.minus(m_poseMeters.getRotation()).getRadians();

        Pose2d newPose1 = m_poseMeters.exp(twist);

        m_previousWheelPositions = wheelPositions.copy();
        // System.out.println("new pose " + newPose1.getX());
        m_poseMeters = new Pose2d(newPose1.getTranslation(), angle);

        Pose2d newPose = m_poseMeters;
        // System.out.println("add sample at time " + currentTimeSeconds);

        m_poseBuffer.addSample(
                currentTimeSeconds,
                new InterpolationRecord(newPose, gyroAngle, wheelPositions.copy()));

        t.log(Level.TRACE, m_name, "GYRO OFFSET", m_gyroOffset);

        return newPose;
    }

    /**
     * Represents an odometry record. The record contains the inputs provided as
     * well as the pose that was observed based on these inputs, as well as the
     * previous record and its inputs.
     */
    private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
        // The pose observed given the current sensor inputs and the previous pose.
        private final Pose2d poseMeters;

        // The current gyro angle.
        private final Rotation2d gyroAngle;

        // The current encoder readings.
        private final SwerveDriveWheelPositions wheelPositions;

        /**
         * Constructs an Interpolation Record with the specified parameters.
         *
         * @param poseMeters     The pose observed given the current sensor inputs and
         *                       the previous pose.
         * @param gyro           The current gyro angle.
         * @param wheelPositions The current encoder readings.
         */
        private InterpolationRecord(
                Pose2d poseMeters,
                Rotation2d gyro,
                SwerveDriveWheelPositions wheelPositions) {
            this.poseMeters = poseMeters;
            this.gyroAngle = gyro;
            this.wheelPositions = wheelPositions;
        }

        /**
         * Return the interpolated record. This object is assumed to be the starting
         * position, or lower
         * bound.
         *
         * @param endValue The upper bound, or end.
         * @param t        How far between the lower and upper bound we are. This should
         *                 be bounded in [0, 1].
         * @return The interpolated value.
         */
        @Override
        public InterpolationRecord interpolate(
                InterpolationRecord endValue, double t) {
            if (t < 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                // Find the new wheel distances.
                SwerveDriveWheelPositions wheelLerp = wheelPositions.interpolate(endValue.wheelPositions, t);

                // Find the new gyro angle.
                Rotation2d gyroLerp = gyroAngle.interpolate(endValue.gyroAngle, t);

                // Create a twist to represent the change based on the interpolated sensor
                // inputs.
                // TODO: this should take tires into account since it modifies the pose
                // estimate.
                Twist2d twist = m_kinematics.toTwist2d(
                        DriveUtil.modulePositionDelta(wheelPositions, wheelLerp));
                twist.dtheta = gyroLerp.minus(gyroAngle).getRadians();

                return new InterpolationRecord(poseMeters.exp(twist), gyroLerp, wheelLerp);
            }
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof InterpolationRecord)) {
                return false;
            }
            InterpolationRecord rec = (InterpolationRecord) obj;
            return Objects.equals(gyroAngle, rec.gyroAngle)
                    && Objects.equals(wheelPositions, rec.wheelPositions)
                    && Objects.equals(poseMeters, rec.poseMeters);
        }

        @Override
        public int hashCode() {
            return Objects.hash(gyroAngle, wheelPositions, poseMeters);
        }
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

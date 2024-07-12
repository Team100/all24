package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.util.Tire;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Kinematics and dynamics of the swerve drive.
 * 
 * Includes speed limits, dynamic constraints, and kinematics.
 * 
 * This class represents *absolute maxima.*
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 */
public class SwerveKinodynamics implements Glassy {
    private final Logger m_logger;

    // geometry
    private final double m_fronttrack;
    private final double m_backtrack;
    private final double m_wheelbase;
    private final double m_frontoffset;
    private final double m_radius;
    private final double m_vcg;
    private final SwerveDriveKinematics100 m_kinematics;
    private final double m_MaxCapsizeAccelM_S2;

    // configured inputs
    private double m_MaxDriveVelocityM_S;
    private double m_StallAccelerationM_S2;
    private double m_MaxDriveAccelerationM_S2;
    private double m_MaxDriveDecelerationM_S2;
    private double m_MaxSteeringVelocityRad_S;
    private double m_maxSteeringAccelerationRad_S2;
    private Tire m_tire;

    // calculated
    private double m_maxAngleSpeedRad_S;
    private double m_maxAngleAccelRad_S2;
    private Profile100 m_steeringProfile;

    /**
     * Use the factory
     * 
     * @param maxDriveVelocity        module drive speed m/s
     * @param stallAcceleration       acceleration at stall, used to compute
     *                                back-EMF-limited acceleration at higher RPMs
     * @param maxDriveAcceleration    module drive accel m/s^2
     * @param maxDriveDeceleration    module drive decel m/s^2. Should be higher
     *                                than accel limit, this is a positive number.
     * @param maxSteeringVelocity     module steering axis rate rad/s
     * @param maxSteeringAcceleration module steering axis accel rad/s^2
     * @param track                   meters
     * @param wheelbase               meters
     * @param frontoffset             distance from front wheels to center of mass,
     *                                meters
     * @param vcg                     vertical center of gravity, meters
     */
    SwerveKinodynamics(
            Logger parent,
            double maxDriveVelocity,
            double stallAcceleration,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxSteeringAcceleration,
            double track,
            double wheelbase,
            double frontoffset,
            double vcg,
            Tire tire) {
        m_logger = parent.child(this);
        if (track < 0.1)
            throw new IllegalArgumentException();
        if (wheelbase < 0.1)
            throw new IllegalArgumentException();

        m_fronttrack = track;
        m_backtrack = track;
        m_wheelbase = wheelbase;
        m_frontoffset = frontoffset;
        m_vcg = vcg;
        m_tire = tire;
        // distance from center to wheel
        m_radius = Math.hypot(track / 2, m_wheelbase / 2);
        m_kinematics = get(m_fronttrack, m_backtrack, m_wheelbase, frontoffset);
        // fulcrum is the distance from the center to the nearest edge.
        double fulcrum = Math.min(m_fronttrack / 2, m_wheelbase / 2);
        m_MaxCapsizeAccelM_S2 = 9.8 * (fulcrum / m_vcg);

        setMaxDriveVelocityM_S(maxDriveVelocity);
        setStallAccelerationM_S2(stallAcceleration);
        setMaxDriveAccelerationM_S2(maxDriveAcceleration);
        setMaxDriveDecelerationM_S2(maxDriveDeceleration);
        setMaxSteeringVelocityRad_S(maxSteeringVelocity);
        setMaxSteeringAccelerationRad_S2(maxSteeringAcceleration);
    }

    /**
     * Use the factory
     * 
     * @param maxDriveVelocity        module drive speed m/s
     * @param stallAcceleration       acceleration at stall, used to compute
     *                                back-EMF-limited acceleration at higher RPMs
     * @param maxDriveAcceleration    module drive accel m/s^2
     * @param maxDriveDeceleration    module drive decel m/s^2. Should be higher
     *                                than accel limit, this is a positive number.
     * @param maxSteeringVelocity     module steering axis rate rad/s
     * @param maxSteeringAcceleration module steering axis accel rad/s^2
     * @param fronttrack              meters
     * @param backtrack               meters
     * @param wheelbase               meters
     * @param vcg                     vertical center of gravity, meters
     */
    SwerveKinodynamics(
            Logger parent,
            double maxDriveVelocity,
            double stallAcceleration,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxSteeringAcceleration,
            double fronttrack,
            double backtrack,
            double wheelbase,
            double frontoffset,
            double vcg,
            Tire tire) {
        m_logger = parent.child(this);
        if (fronttrack < 0.1 || backtrack < 0.1)
            throw new IllegalArgumentException();
        if (wheelbase < 0.1)
            throw new IllegalArgumentException();

        m_fronttrack = fronttrack;
        m_backtrack = backtrack;
        m_wheelbase = wheelbase;
        m_frontoffset = frontoffset;
        m_vcg = vcg;
        m_tire = tire;
        // distance from center to wheel
        m_radius = Math.hypot((fronttrack + backtrack) / 4, m_wheelbase / 2);
        m_kinematics = get(m_fronttrack, m_backtrack, m_wheelbase, m_frontoffset);
        // fulcrum is the distance from the center to the nearest edge.
        double fulcrum = Math.min(m_fronttrack / 2, m_wheelbase / 2);
        m_MaxCapsizeAccelM_S2 = 9.8 * (fulcrum / m_vcg);

        setMaxDriveVelocityM_S(maxDriveVelocity);
        setStallAccelerationM_S2(stallAcceleration);
        setMaxDriveAccelerationM_S2(maxDriveAcceleration);
        setMaxDriveDecelerationM_S2(maxDriveDeceleration);
        setMaxSteeringVelocityRad_S(maxSteeringVelocity);
        setMaxSteeringAccelerationRad_S2(maxSteeringAcceleration);
    }

    private void setMaxDriveVelocityM_S(double maxDriveVelocityM_S) {
        m_MaxDriveVelocityM_S = maxDriveVelocityM_S;
        setAngleSpeed();
    }

    private void setStallAccelerationM_S2(double stallAccelerationM_S2) {
        m_StallAccelerationM_S2 = stallAccelerationM_S2;
    }

    private void setMaxDriveAccelerationM_S2(double maxDriveAccelerationM_S2) {
        m_MaxDriveAccelerationM_S2 = maxDriveAccelerationM_S2;
        setAngleAccel();
    }

    private void setMaxDriveDecelerationM_S2(double maxDriveDecelerationM_S2) {
        m_MaxDriveDecelerationM_S2 = maxDriveDecelerationM_S2;
        setAngleAccel();
    }

    private void setMaxSteeringVelocityRad_S(double maxSteeringVelocityRad_S) {
        m_MaxSteeringVelocityRad_S = maxSteeringVelocityRad_S;
        setSteeringProfile();
    }

    private void setMaxSteeringAccelerationRad_S2(double maxSteeringAccelerationRad_S2) {
        m_maxSteeringAccelerationRad_S2 = maxSteeringAccelerationRad_S2;
        setSteeringProfile();
    }

    private void setAngleSpeed() {
        m_maxAngleSpeedRad_S = m_MaxDriveVelocityM_S / m_radius;
    }

    private void setAngleAccel() {
        // this assumes the robot is a uniform rectangular cuboid.
        double accel = Math.max(m_MaxDriveAccelerationM_S2, m_MaxDriveDecelerationM_S2);
        m_maxAngleAccelRad_S2 = 12 * accel * m_radius
                / (m_fronttrack * m_fronttrack + m_wheelbase * m_wheelbase);
    }

    private void setSteeringProfile() {
        m_steeringProfile = new TrapezoidProfile100(
                m_MaxSteeringVelocityRad_S,
                m_maxSteeringAccelerationRad_S2,
                0.02);
    }

    public Profile100 getSteeringProfile() {
        return m_steeringProfile;
    }

    /** Cruise speed, m/s. */
    public double getMaxDriveVelocityM_S() {
        return m_MaxDriveVelocityM_S;
    }

    /**
     * Acceleration at stall, without current limiting. Used to compute
     * back-EMF-limited torque available at higher RPMs.
     */
    public double getStallAccelerationM_S2() {
        return m_StallAccelerationM_S2;
    }

    /** Motor-torque-limited acceleration rate, m/s^2 */
    public double getMaxDriveAccelerationM_S2() {
        return m_MaxDriveAccelerationM_S2;
    }

    /**
     * Motor-torque-limited drive deceleration rate, m/s^2. Motors are better at
     * slowing down than speeding up, so this should be larger than the accel rate.
     */
    public double getMaxDriveDecelerationM_S2() {
        return m_MaxDriveDecelerationM_S2;
    }

    /** Cruise speed of the swerve steering axes, rad/s. */
    public double getMaxSteeringVelocityRad_S() {
        return m_MaxSteeringVelocityRad_S;
    }

    /** Cruise speed of the swerve steering axes, rad/s. */
    public double getMaxSteeringAccelRad_S2() {
        return m_maxSteeringAccelerationRad_S2;
    }

    /** Spin cruise speed, rad/s. Computed from drive and frame size. */
    public double getMaxAngleSpeedRad_S() {
        return m_maxAngleSpeedRad_S;
    }

    /**
     * Motor-torque-limited spin accel rate, rad/s^2. Computed from drive and frame
     * size.
     */
    public double getMaxAngleAccelRad_S2() {
        return m_maxAngleAccelRad_S2;
    }

    /**
     * Acceleration which will tip the robot onto two wheels, m/s^2. Computed from
     * vertical center of gravity and frame size.
     */
    public double getMaxCapsizeAccelM_S2() {
        return m_MaxCapsizeAccelM_S2;
    }

    /**
     * @param fronttrack
     * @param backtrack
     * @param wheelbase
     * @param frontoffset distance from center of mass to front wheel
     * @return
     */
    private static SwerveDriveKinematics100 get(double fronttrack, double backtrack, double wheelbase,
            double frontoffset) {
        return new SwerveDriveKinematics100(
                new Translation2d(frontoffset, fronttrack / 2),
                new Translation2d(frontoffset, -fronttrack / 2),
                new Translation2d(frontoffset - wheelbase, backtrack / 2),
                new Translation2d(frontoffset - wheelbase, -backtrack / 2));
    }

    /** arg elements are nullable */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        m_kinematics.resetHeadings(moduleHeadings);
    }

    /**
     * Inverse kinematics, chassis speeds => module states.
     * 
     * The resulting state speeds are always positive.
     * 
     * This version does **DISCRETIZATION** to correct for swerve veering.
     * 
     * It also does extra veering correction proportional to rotation rate and
     * translational acceleration.
     * 
     * @param in            chassis speeds to transform
     * @param gyroRateRad_S current gyro rate, or the trajectory gyro rate
     * @param dt            time to aim for
     */
    public SwerveModuleState100[] toSwerveModuleStates(ChassisSpeeds in, double gyroRateRad_S, double dt) {
        // This is the extra correction angle ...
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(gyroRateRad_S));
        // ... which is subtracted here; this isn't really a field-relative
        // transformation it's just a rotation.
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                in.vxMetersPerSecond,
                in.vyMetersPerSecond,
                in.omegaRadiansPerSecond,
                angle);
        ChassisSpeeds descretized = ChassisSpeeds.discretize(chassisSpeeds, dt);
        return m_kinematics.toSwerveModuleStates(descretized);
    }

    /**
     * Inverse kinematics, chassis speeds => module states.
     * 
     * The resulting state speeds are always positive.
     * 
     * This version does **DISCRETIZATION** to correct for swerve veering.
     * 
     * It also does extra veering correction proportional to rotation rate and
     * translational acceleration.
     * 
     * @param in            chassis speeds to transform
     * @param gyroRateRad_S current gyro rate, or the trajectory gyro rate
     * @param accelM_S      magnitude of acceleration
     * @param dt            time to aim for
     */
    public SwerveModuleState100[] toSwerveModuleStates(ChassisSpeeds in, ChassisSpeeds prevIn, SwerveModuleState100[] prevSwerveModuleState, double gyroRateRad_S, double dt) {
        // This is the extra correction angle ...
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(gyroRateRad_S));
        // ... which is subtracted here; this isn't really a field-relative
        // transformation it's just a rotation.
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                in.vxMetersPerSecond,
                in.vyMetersPerSecond,
                in.omegaRadiansPerSecond,
                angle);
        ChassisSpeeds descretized = ChassisSpeeds.discretize(chassisSpeeds, dt);
        
        ChassisSpeeds prevChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                prevIn.vxMetersPerSecond,
                prevIn.vyMetersPerSecond,
                prevIn.omegaRadiansPerSecond,
                angle);
        ChassisSpeeds prevDescretized = ChassisSpeeds.discretize(prevChassisSpeeds, dt);
        
        ChassisSpeeds acceleration = (chassisSpeeds.minus(prevDescretized)).div(dt);

        return m_kinematics.toSwerveModuleStates(descretized, acceleration, prevSwerveModuleState, dt);
    }

    /**
     * The resulting state speeds are always positive.
     */
    public SwerveModuleState100[] toSwerveModuleStatesWithoutDiscretization(ChassisSpeeds speeds, ChassisSpeeds prevChassisSpeeds, SwerveModuleState100[] prevModuleStates, double dt) {
        return m_kinematics.toSwerveModuleStates(speeds, prevChassisSpeeds, prevModuleStates, dt);
    }

    /**
     * The resulting state speeds are always positive.
     */
    public SwerveModuleState100[] toSwerveModuleStatesWithoutDiscretization(ChassisSpeeds speeds) {
        return m_kinematics.toSwerveModuleStates(speeds);
    }

    /**
     * Forward kinematics, module states => chassis speeds.
     * 
     * Does not do inverse discretization.
     * 
     * Does not take Tires into account.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState100... moduleStates) {
        // does not take tires into account
        return m_kinematics.toChassisSpeeds(moduleStates);
    }

    /**
     * This could be used with odometry, but because odometry uses module positions
     * instead of velocities, it is not needed.
     * 
     * It performs inverse discretization and an extra correction.
     * 
     * Does not take Tires into account.
     */
    public ChassisSpeeds toChassisSpeedsWithDiscretization(
            double gyroRateRad_S,
            double dt,
            SwerveModuleState100... moduleStates) {
        ChassisSpeeds discreteSpeeds = toChassisSpeeds(moduleStates);
        Twist2d twist = new Twist2d(
                discreteSpeeds.vxMetersPerSecond * dt,
                discreteSpeeds.vyMetersPerSecond * dt,
                discreteSpeeds.omegaRadiansPerSecond * dt);

        Pose2d deltaPose = GeometryUtil.sexp(twist);
        ChassisSpeeds continuousSpeeds = new ChassisSpeeds(
                deltaPose.getX(),
                deltaPose.getY(),
                deltaPose.getRotation().getRadians()).div(dt);

        // This is the opposite direction
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(gyroRateRad_S));
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                continuousSpeeds.vxMetersPerSecond,
                continuousSpeeds.vyMetersPerSecond,
                continuousSpeeds.omegaRadiansPerSecond,
                angle.unaryMinus());
    }

    public SwerveDrivePoseEstimator100 newPoseEstimator(
            Rotation2d gyroAngle,
            SwerveModulePosition100[] modulePositions,
            Pose2d initialPoseMeters,
            double timestampSeconds) {
        return new SwerveDrivePoseEstimator100(
                m_logger,
                this,
                gyroAngle,
                modulePositions,
                initialPoseMeters,
                timestampSeconds);
    }

    /**
     * Maintain translation and rotation proportionality but slow to a feasible
     * velocity, assuming the robot has an infinite number of wheels on a circular
     * frame.
     */
    public ChassisSpeeds analyticDesaturation(ChassisSpeeds speeds) {
        double maxV = getMaxDriveVelocityM_S();
        double maxOmega = getMaxAngleSpeedRad_S();
        double xySpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double xyAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.omegaRadiansPerSecond) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // if we got here then omega alone is infeasible so use maxomega
            return new ChassisSpeeds(0, 0, Math.signum(speeds.omegaRadiansPerSecond) * maxOmega);
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) < 1e-12) {
            // if we got here then xyspeed alone is infeasible so use maxV
            return new ChassisSpeeds(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.omegaRadiansPerSecond) * maxV);

        double vRatio = v / xySpeed;

        return new ChassisSpeeds(
                vRatio * speeds.vxMetersPerSecond,
                vRatio * speeds.vyMetersPerSecond,
                vRatio * speeds.omegaRadiansPerSecond);
    }

    /**
     * Input could be field-relative or robot-relative, the math doesn't depend on
     * theta, because it treats the robot like a circle.
     * 
     * Input must be full-scale, meters/sec and radians/sec, this won't work on
     * control units [-1,1].
     * 
     * @param speeds twist in m/s and rad/s
     * @return
     */
    public FieldRelativeVelocity analyticDesaturation(FieldRelativeVelocity speeds) {
        double maxV = getMaxDriveVelocityM_S();
        double maxOmega = getMaxAngleSpeedRad_S();
        double xySpeed = speeds.norm();
        double xyAngle = Math.atan2(speeds.y(), speeds.x());
        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        if (Math.abs(speeds.theta()) <= omegaForSpeed) {
            // omega + xyspeed is feasible
            return speeds;
        }
        if (xySpeed < 1e-12) {
            // if we got here then omega alone is infeasible so use maxomega
            return new FieldRelativeVelocity(0, 0, Math.signum(speeds.theta()) * maxOmega);
        }
        if (Math.abs(speeds.theta()) < 1e-12) {
            // if we got here then xyspeed alone is infeasible so use maxV
            return new FieldRelativeVelocity(maxV * Math.cos(xyAngle), maxV * Math.sin(xyAngle), 0);
        }

        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(speeds.theta()) * maxV);

        double vRatio = v / xySpeed;

        return new FieldRelativeVelocity(
                vRatio * speeds.x(),
                vRatio * speeds.y(),
                vRatio * speeds.theta());
    }

    /** Scales translation to accommodate the rotation. */
    public ChassisSpeeds preferRotation(ChassisSpeeds speeds) {
        double oRatio = Math.min(1, speeds.omegaRadiansPerSecond / getMaxAngleSpeedRad_S());
        double xySpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double maxV = getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - oRatio, xyRatio);
        double xyAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

        return new ChassisSpeeds(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.omegaRadiansPerSecond);
    }

    /** Scales translation to accommodate the rotation. */
    public FieldRelativeVelocity preferRotation(FieldRelativeVelocity speeds) {
        double oRatio = Math.min(1, speeds.theta() / getMaxAngleSpeedRad_S());
        double xySpeed = speeds.norm();
        double maxV = getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - oRatio, xyRatio);
        double xyAngle = Math.atan2(speeds.y(), speeds.x());

        return new FieldRelativeVelocity(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.theta());
    }

    public Tire getTire() {
        return m_tire;
    }

    public SwerveDriveKinematics100 getKinematics() {
        return m_kinematics;
    }

    @Override
    public String getGlassName() {
        return "SwerveKinodynamics";
    }

}
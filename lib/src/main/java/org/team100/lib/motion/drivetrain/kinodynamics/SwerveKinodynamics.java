package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.profile.ChoosableProfile;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
public class SwerveKinodynamics {

    // configured inputs
    private final double m_MaxDriveVelocityM_S;
    private final double m_MaxDriveAccelerationM_S2;
    private final double m_MaxDriveDecelerationM_S2;
    private final double m_MaxSteeringVelocityRad_S;

    // calculated
    private final double m_maxAngleSpeedRad_S;
    private final double m_maxAngleAccelRad_S2;
    private final double m_MaxCapsizeAccelM_S2;
    private final SwerveDriveKinematics m_kinematics;
    private final ChoosableProfile m_steeringProfile;

    /**
     * Use the factory
     * 
     * @param maxDriveVelocity        module drive speed m/s
     * @param maxDriveAcceleration    module drive accel m/s^2
     * @param maxDriveDeceleration    module drive decel m/s^2. Should be higher
     *                                than
     *                                accel limit.
     * @param maxSteeringVelocity     module steering axis rate rad/s
     * @param maxSteeringAcceleration module steering axis accel rad/s^2
     * @param track                   meters
     * @param wheelbase               meters
     * @param vcg                     vertical center of gravity, meters
     */
    SwerveKinodynamics(
            double maxDriveVelocity,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxSteeringAcceleration,
            double track,
            double wheelbase,
            double vcg) {

        m_MaxDriveVelocityM_S = maxDriveVelocity;
        m_MaxDriveAccelerationM_S2 = maxDriveAcceleration;
        m_MaxDriveDecelerationM_S2 = maxDriveDeceleration;
        m_MaxSteeringVelocityRad_S = maxSteeringVelocity;

        // distance from center to wheel
        double radius = Math.hypot(track / 2, wheelbase / 2);
        m_maxAngleSpeedRad_S = m_MaxDriveVelocityM_S / radius;
        // this assumes the robot is a uniform rectangular cuboid.
        m_maxAngleAccelRad_S2 = 12 * m_MaxDriveAccelerationM_S2 * radius
                / (track * track + wheelbase * wheelbase);

        // fulcrum is the distance from the center to the nearest edge.
        double fulcrum = Math.min(track / 2, wheelbase / 2);
        m_MaxCapsizeAccelM_S2 = 9.8 * (fulcrum / vcg);

        m_kinematics = get(track, wheelbase);
        m_steeringProfile = new ChoosableProfile(
                maxSteeringVelocity,
                maxSteeringAcceleration,
                ChoosableProfile.Mode.TRAPEZOID);
    }

    public ChoosableProfile getSteeringProfile() {
        return m_steeringProfile;
    }

    /** Cruise speed, m/s. */
    public double getMaxDriveVelocityM_S() {
        return m_MaxDriveVelocityM_S;
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

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /** If you want to rotate the robot with a trapezoidal profile, use this. */
    public TrapezoidProfile.Constraints getAngleConstraints() {
        return new TrapezoidProfile.Constraints(
                getMaxAngleSpeedRad_S(),
                getMaxAngleAccelRad_S2());
    }

    /** Trapezoidal profile for linear motion. */
    public TrapezoidProfile.Constraints getDistanceConstraints() {
        return new TrapezoidProfile.Constraints(
                getMaxDriveVelocityM_S(),
                getMaxDriveAccelerationM_S2());
    }

    private static SwerveDriveKinematics get(double track, double wheelbase) {
        return new SwerveDriveKinematics(
                new Translation2d(wheelbase / 2, track / 2),
                new Translation2d(wheelbase / 2, -track / 2),
                new Translation2d(-wheelbase / 2, track / 2),
                new Translation2d(-wheelbase / 2, -track / 2));
    }

}

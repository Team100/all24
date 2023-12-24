package org.team100.lib.motion.drivetrain.kinodynamics;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Kinematics and dynamics of the swerve drive.
 * 
 * Includes speed limits, dynamic constraints, and kinematics.
 * 
 * Motors are motors are better at slowing down than speeding up, so drive accel
 * and decel are included separately.
 * 
 * The capsize limit is also a separate acceleration.
 * 
 * TODO: calculate angular accel/speed based on wheelbase.
 * 
 * TODO make this center of gravity height instead, use wheelbase and track and
 * gravity to produce *capsize* accel limit, sincei t applies in line not just
 * cross track
 * 
 * TODO make a drvedynamics class that also includes kinematics and whelbase and
 * the other speedlimits thing
 */
public class SwerveKinodynamics {

    private final double m_maxSpeedM_S;
    private final double m_maxAccelM_S2;
    // TODO: derive this from dimensions
    private final double m_maxAngleSpeedRad_S;
    // TODO: derive this from mass and dimensions
    private final double m_maxAngleAccelRad_S2;

    // TODO: dedupe these

    private final double m_MaxDriveVelocity; // m/s
    private final double m_MaxDriveAcceleration; // m/s^2
    private final double m_MaxDriveDeceleration; // m/s^2
    private final double m_MaxSteeringVelocity; // rad/s
    private final double m_MaxCapsizeAccel; // m/s^2

    // TODO: add show mode here as a multiplier.

    /** Use the factory */
    SwerveKinodynamics(
            double maxSpeedM_S,
            double maxAccelM_S2,
            double maxAngleSpeedRad_S,
            double maxAngleAccelRad_S2,

            double maxDriveVelocity,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxCapsizeAccel

    ) {
        m_maxSpeedM_S = maxSpeedM_S;
        m_maxAccelM_S2 = maxAccelM_S2;
        m_maxAngleSpeedRad_S = maxAngleSpeedRad_S;
        m_maxAngleAccelRad_S2 = maxAngleAccelRad_S2;

        m_MaxDriveVelocity = maxDriveVelocity;
        m_MaxDriveAcceleration = maxDriveAcceleration;
        m_MaxDriveDeceleration = maxDriveDeceleration;
        m_MaxSteeringVelocity = maxSteeringVelocity;
        m_MaxCapsizeAccel = maxCapsizeAccel;
    }

    public TrapezoidProfile.Constraints getAngleConstraints() {
        return new TrapezoidProfile.Constraints(
                getMaxAngleSpeedRad_S(),
                getMaxAngleAccelRad_S2());
    }

    public TrapezoidProfile.Constraints getDistanceConstraints() {
        return new TrapezoidProfile.Constraints(
                getMaxSpeedM_S(),
                getMaxAccelM_S2());
    }

    public double getMaxSpeedM_S() {
        return m_maxSpeedM_S;
    }

    public double getMaxAccelM_S2() {
        return m_maxAccelM_S2;
    }

    public double getMaxAngleSpeedRad_S() {
        return m_maxAngleSpeedRad_S;
    }

    public double getMaxAngleAccelRad_S2() {
        return m_maxAngleAccelRad_S2;
    }

    public double getMaxDriveVelocity() {
        return m_MaxDriveVelocity;
    }

    public double getMaxDriveAcceleration() {
        return m_MaxDriveAcceleration;
    }

    public double getMaxDriveDeceleration() {
        return m_MaxDriveDeceleration;
    }

    public double getMaxSteeringVelocity() {
        return m_MaxSteeringVelocity;
    }

    public double getMaxCapsizeAccel() {
        return m_MaxCapsizeAccel;
    }

}

package org.team100.lib.sensors;

import org.team100.lib.motion.drivetrain.SwerveModuleCollection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/**
 * A simulated gyro that uses drivetrain odometry.
 */
public class SimulatedHeading implements HeadingInterface {
    private double m_heading = 0;
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveModuleCollection m_moduleCollection;
    private double m_time = Timer.getFPGATimestamp();

    public SimulatedHeading(
            SwerveDriveKinematics kinematics,
            SwerveModuleCollection collection) {
        m_kinematics = kinematics;
        m_moduleCollection = collection;
    }

    @Override
    public Rotation2d getHeadingNWU() {
        SwerveModuleState[] states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(states);
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        m_heading += speeds.omegaRadiansPerSecond * dt;
        m_time = now;
        return new Rotation2d(m_heading);
    }

    @Override
    public double getHeadingRateNWU() {
        // only used for veering, ignore for now
        return 0;
    }

}

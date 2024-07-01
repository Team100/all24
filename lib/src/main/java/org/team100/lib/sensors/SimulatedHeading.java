package org.team100.lib.sensors;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * A simulated gyro that uses drivetrain odometry.
 * 
 * Does not use the tire model.
 */
public class SimulatedHeading implements HeadingInterface {
    private double m_heading = 0;
    private final SwerveKinodynamics m_kinodynamics;
    private final SwerveModuleCollection m_moduleCollection;
    private double m_time = Timer.getFPGATimestamp();

    public SimulatedHeading(
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        m_kinodynamics = kinodynamics;
        m_moduleCollection = collection;
    }

    @Override
    public Rotation2d getHeadingNWU() {
        SwerveModuleState[] states = m_moduleCollection.states();
        // discretization is not necessary here because we only use the rotation, which
        // is invariant
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeeds(states);
        double now = Timer.getFPGATimestamp();
        double dt = now - m_time;
        m_heading += speeds.omegaRadiansPerSecond * dt;
        m_time = now;
        return new Rotation2d(m_heading);
    }

    @Override
    public double getHeadingRateNWU() {
        SwerveModuleState[] states = m_moduleCollection.states();
        // discretization is not necessary here because we only use the rotation, which
        // is invariant
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeeds(states);
        return speeds.omegaRadiansPerSecond;
    }

}

package org.team100.lib.swerve;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Used to track setpoints over time and enforce limits.
 */
public class SwerveSetpoint {
    private final ChassisSpeeds m_ChassisSpeeds;
    private final SwerveModuleStates m_ModuleStates;

    /** New setpoint with zero speed and indeterminate steering */
    public SwerveSetpoint() {
        this(
                new ChassisSpeeds(),
                new SwerveModuleStates(
                        new SwerveModuleState100(0, Optional.empty()),
                        new SwerveModuleState100(0, Optional.empty()),
                        new SwerveModuleState100(0, Optional.empty()),
                        new SwerveModuleState100(0, Optional.empty())));
    }

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleStates initialStates) {
        m_ChassisSpeeds = chassisSpeeds;
        m_ModuleStates = initialStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_ChassisSpeeds;
    }

    public SwerveModuleStates getModuleStates() {
        return m_ModuleStates;
    }

    @Override
    public String toString() {
        String ret = m_ChassisSpeeds.toString() + "\n";
        ret += "  " + m_ModuleStates.toString() + "\n";
        return ret;
    }
}

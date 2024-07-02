package org.team100.lib.swerve;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** 
 * Used to track setpoints over time and enforce limits.
 */
public class SwerveSetpoint {
    private final ChassisSpeeds m_ChassisSpeeds;
    private final SwerveModuleState100[] m_ModuleStates;

    /** New setpoint with zero speed and zero states */
    public SwerveSetpoint() {
        this(
                new ChassisSpeeds(),
                new SwerveModuleState100[] {
                        new SwerveModuleState100(0, GeometryUtil.kRotationZero),
                        new SwerveModuleState100(0, GeometryUtil.kRotationZero),
                        new SwerveModuleState100(0, GeometryUtil.kRotationZero),
                        new SwerveModuleState100(0, GeometryUtil.kRotationZero)
                });
    }

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState100[] initialStates) {
        m_ChassisSpeeds = chassisSpeeds;
        m_ModuleStates = initialStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_ChassisSpeeds;
    }

    public SwerveModuleState100[] getModuleStates() {
        return m_ModuleStates;
    }

    @Override
    public String toString() {
        String ret = m_ChassisSpeeds.toString() + "\n";
        for (int i = 0; i < m_ModuleStates.length; ++i) {
            ret += "  " + m_ModuleStates[i].toString() + "\n";
        }
        return ret;
    }
}

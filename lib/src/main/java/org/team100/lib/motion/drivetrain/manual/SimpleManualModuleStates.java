package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.commands.drivetrain.ModuleStateDriver;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Transform manual input into swerve module states, in a simpler way than
 * ManualModuleStates does.
 * 
 * The input dtheta is exactly the module angle.
 * The input dx is exactly the wheel speed.
 * The input dy is ignored.
 */
public class SimpleManualModuleStates implements ModuleStateDriver {
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final String m_name;

    public SimpleManualModuleStates(String parent, SwerveKinodynamics swerveKinodynamics) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_name = Names.append(parent, this);
    }

    /**
     * There's no conflict between translation and rotation velocities in this mode.
     */
    @Override
    public SwerveModuleState[] apply(Twist2d input) {
        // dtheta is from [-1, 1], so angle is [-pi, pi]
        Rotation2d angle = Rotation2d.fromRadians(Math.PI * input.dtheta);
        double speedM_S = m_swerveKinodynamics.getMaxDriveVelocityM_S() * input.dx;
        t.log(Level.DEBUG, m_name, "speed m_s", speedM_S);
        t.log(Level.DEBUG, m_name, "angle rad", angle.getRadians());
        return new SwerveModuleState[] {
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle),
                new SwerveModuleState(speedM_S, angle)
        };
    }
}

package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.CameraAngles;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
/**
 * Manual no field relative driving while robot rotation is controlled to face note
 * 
 * The x and y are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class ManualWithNoteRotation {
    private final PIDController m_pidController;
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final String m_name;
    private final CameraAngles m_noteCamera;
    public ManualWithNoteRotation(String parent, SwerveKinodynamics swerveKinodynamics, PIDController pidController, CameraAngles noteCamera) {
        m_noteCamera = noteCamera; 
        m_pidController = pidController;
        m_swerveKinodynamics = swerveKinodynamics;
        m_name = Names.append(parent, this);
    }

    /**
     * Changes input to ignore rotational input
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     *
     * @param input in control units, [-1,1]
     * @return feasible chassis speeds in m/s and rad/s
     */
    public ChassisSpeeds apply(Twist2d input) {
        // clip the input to the unit circle
        double dtheta;
        // System.out.println("EE");
        if (m_noteCamera.getY() == null) {
            dtheta = 0;
        } else {
            dtheta = m_noteCamera.getY();
        }

        Twist2d clipped = DriveUtil.clampTwist(new Twist2d(input.dx,input.dy,m_pidController.calculate(dtheta, 
        0)), 1.0);
        // scale to max in both translation and rotation
        t.log(Level.DEBUG, m_name, "twist", clipped);
        Twist2d preferRot = m_swerveKinodynamics.preferRotation(clipped);
        ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                preferRot,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // desaturate to feasibility
        t.log(Level.DEBUG, m_name, "scaled", scaled);
        ChassisSpeeds speeds = m_swerveKinodynamics.analyticDesaturation(scaled);
        t.log(Level.DEBUG, m_name, "speeds", speeds);
        return scaled;
    }
}

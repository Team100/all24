package org.team100.lib.motion.drivetrain.manual;

import java.util.Optional;

import org.team100.lib.commands.drivetrain.ChassisSpeedDriver;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Manual no field relative driving while robot rotation is controlled to face
 * note
 * 
 * The x and y are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class DriveWithNoteRotation implements ChassisSpeedDriver {
    private final PIDController m_pidController;
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final String m_name;
    private final NotePosition24ArrayListener m_arrayListener;

    public DriveWithNoteRotation(String parent, SwerveKinodynamics swerveKinodynamics, PIDController pidController,
            NotePosition24ArrayListener arrayListener) {
        m_arrayListener = arrayListener;
        m_pidController = pidController;
        m_swerveKinodynamics = swerveKinodynamics;
        m_name = Names.append(parent, this);
    }

    /**
     * Changes input to ignore rotational input
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     */
    @Override
    public ChassisSpeeds apply(Twist2d input) {
        // clip the input to the unit circle
        Optional<Double> x = m_arrayListener.getX();
        if (!x.isPresent()) return new ChassisSpeeds();
        Twist2d clipped = DriveUtil.clampTwist(
                new Twist2d(input.dx, input.dy, m_pidController.calculate(x.get(), 416)), 1.0);
        // scale to max in both translation and rotation

        ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // desaturate to feasibility
        ChassisSpeeds speeds = m_swerveKinodynamics.analyticDesaturation(scaled);
        t.log(Level.DEBUG, m_name, "speeds", speeds);
        return speeds;
    }

    @Override
    public void reset(Pose2d p) {
        //
    }
}

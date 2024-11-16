package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Uses a holonomic profile. */
public class OscillateProfile extends Command implements Glassy {
    private static final double TOLERANCE = 0.05;

    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicProfile m_profile;
    private final HolonomicFieldRelativeController m_controller;
    private final double m_offsetM;

    private SwerveControl m_setpoint;
    private SwerveModel m_goal;

    public OscillateProfile(
            SwerveDriveSubsystem swerve,
            HolonomicProfile profile,
            HolonomicFieldRelativeController controller,
            double offsetM) {
        m_swerve = swerve;
        m_profile = profile;
        m_controller = controller;
        m_offsetM = offsetM;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_swerve.stop();
        m_controller.reset();
        // choose a goal 1m away
        SwerveModel start = m_swerve.getState();
        Pose2d startPose = start.pose();
        // don't rotate
        Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0, new Rotation2d()));
        // spin 180 between the endpoints
        // Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0,
        // GeometryUtil.kRotation180));
        m_goal = new SwerveModel(endPose);
        m_setpoint = start.control();
        m_profile.solve(m_setpoint.model(), m_goal);
    }

    @Override
    public void execute() {
        SwerveModel measurement = m_swerve.getState();
        m_setpoint = m_profile.calculate(m_setpoint.model(), m_goal);
        // System.out.println("measurement " + measurement + " setpoint " + m_setpoint);
        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, m_setpoint.model());
        // System.out.println(fieldRelativeTarget);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);
    }

    @Override
    public boolean isFinished() {
        SwerveModel measurement = m_swerve.getState();
        return measurement.near(m_goal, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}

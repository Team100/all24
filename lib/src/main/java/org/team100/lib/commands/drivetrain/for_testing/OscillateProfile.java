package org.team100.lib.commands.drivetrain.for_testing;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
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

    private SwerveState m_setpoint;
    private SwerveState m_goal;

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
        SwerveState start = m_swerve.getState();
        Pose2d startPose = start.pose();
        // don't rotate
        Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0, new Rotation2d()));
        // spin 180 between the endpoints
        // Pose2d endPose = startPose.plus(new Transform2d(m_offsetM, 0,
        // GeometryUtil.kRotation180));
        m_goal = new SwerveState(endPose);
        m_setpoint = start;
        m_profile.solve(m_setpoint, m_goal);
    }

    @Override
    public void execute() {
        SwerveState measurement = m_swerve.getState();
        // TODO: make this actually work
        // adjust the rotation in the goal so that it points at the setpoint
        // the idea is to arrive at the goal facing it.
        // m_goal = m_goal.withTheta(m_goal.translation().minus(m_setpoint.translation()).getAngle().getRadians());
        // this will change the profile, though, since rotation can be the slowest part.
        // m_profile.solve(m_setpoint, m_goal);

        m_setpoint = m_profile.calculate(m_setpoint, m_goal);
        // System.out.println("measurement " + measurement + " setpoint " + m_setpoint);
        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, m_setpoint);
        // System.out.println(fieldRelativeTarget);
        m_swerve.driveInFieldCoords(fieldRelativeTarget);
    }

    @Override
    public boolean isFinished() {
        SwerveState measurement = m_swerve.getState();
        return measurement.near(m_goal, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}

package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveUnderProfile extends Command100 {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private final Transform2d m_robotRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicDriveController3 m_controller;
    private final SwerveKinodynamics m_limits;
    private final TrapezoidProfile100 xProfile;
    private final TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;

    /**
     * @param goal
     * @param drivetrain
     * @param planner
     * @param controller
     * @param viz        ok to be null
     */
    public DriveUnderProfile(
            Transform2d robotRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicDriveController3 controller,
            SwerveKinodynamics limits) {
        m_robotRelativeGoal = robotRelativeGoal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_limits = limits;
        Constraints100 thetaContraints = new Constraints100(m_limits.getMaxAngleSpeedRad_S(),m_limits.getMaxAngleAccelRad_S2());
        Constraints100 driveContraints = new Constraints100(m_limits.getMaxDriveVelocityM_S(),m_limits.getMaxDriveAccelerationM_S2());
        xProfile = new TrapezoidProfile100(driveContraints, 0.01);
        yProfile = new TrapezoidProfile100(driveContraints, 0.01);
        thetaProfile = new TrapezoidProfile100(thetaContraints, 0.01);
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
    }

    @Override
    public void execute100(double dt) {
        Pose2d fieldRelativeGoal = m_swerve.getPose().transformBy(m_robotRelativeGoal);
        SwerveState state = m_swerve.getState();
        State100 xState = xProfile.calculate(0.02, state.x(), new State100(fieldRelativeGoal.getX(),0,0));
        State100 yState = yProfile.calculate(0.02, state.y(), new State100(fieldRelativeGoal.getY(),0,0));
        State100 thetaState = thetaProfile.calculate(0.02, state.theta(), new State100(fieldRelativeGoal.getRotation().getRadians(),0,0));
        SwerveState goalState = new SwerveState(xState, yState, thetaState);
        Twist2d goal = m_controller.calculate(m_swerve.getPose(),goalState);
        m_swerve.driveInFieldCoords(goal, 0.02);    
    }

    @Override
    public boolean isFinished() {
        return m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}

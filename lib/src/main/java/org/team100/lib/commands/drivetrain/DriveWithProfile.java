package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.Constraints100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveWithProfile extends Command100 {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private final Supplier<Pose2d> m_robotRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicDriveController3 m_controller;
    private final SwerveKinodynamics m_limits;
    private final TrapezoidProfile100 xProfile;
    private final TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;
    private State100 xSetpoint;
    private State100 ySetpoint;
    private State100 thetaSetpoint;
    /**
     * @param goal
     * @param drivetrain
     * @param controller
     * @param limits
     */
    public DriveWithProfile(
            Supplier<Pose2d> robotRelativeGoal,
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
        xSetpoint = m_swerve.getState().x();
        ySetpoint = m_swerve.getState().y();
        thetaSetpoint = m_swerve.getState().theta();
    }

    @Override
    public void execute100(double dt) {
        if (m_robotRelativeGoal.get() != null) {
            System.out.println("WORKIGN");
        Rotation2d currentRotation = m_swerve.getPose().getRotation();
        // take the short path
        double measurement = currentRotation.getRadians();
        Rotation2d bearing = new Rotation2d(
                Math100.getMinDistance(measurement, m_robotRelativeGoal.get().getRotation().getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, thetaSetpoint.x()),
                thetaSetpoint.v());
        State100 thetaGoal = new State100(bearing.getRadians(), 0);
        State100 xGoalRaw = new State100(m_robotRelativeGoal.get().getX(),0,0);
        xSetpoint = xProfile.calculate(0.02, xSetpoint, xGoalRaw);
        State100 yGoalRaw = new State100(m_robotRelativeGoal.get().getY(),0,0);
        ySetpoint = yProfile.calculate(0.02, ySetpoint, yGoalRaw);
        // State100 thetaGoalRaw = new State100(m_robotRelativeGoal.get().getRotation().getRadians(),0,0);
        thetaSetpoint = thetaProfile.calculate(0.02, thetaSetpoint, thetaGoal);
        SwerveState goalState = new SwerveState(xSetpoint, ySetpoint, thetaSetpoint);
        Twist2d goal = m_controller.calculate(m_swerve.getPose(), goalState);
        m_swerve.driveInFieldCoords(goal, 0.02);
        } else {
            System.out.println("Detection error");
            m_swerve.driveInFieldCoords(new Twist2d(), 0.02);
        }    
    }

    @Override
    public boolean isFinished() {
        //TODO make this end when intake detects note intake
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}

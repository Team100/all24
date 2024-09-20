package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.controller.State100;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 * 
 * Sanjans version
 */
public class DriveWithProfile2 extends Command100 {
    private static final double kTolerance = 0.1;
    private final Supplier<Optional<Pose2d>> m_fieldRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicDriveController100 m_controller;
    private final SwerveKinodynamics m_limits;
    private final TrapezoidProfile100 xProfile;
    private final TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;

    private State100 xSetpoint;
    private State100 ySetpoint;
    private State100 thetaSetpoint;

    private State100 m_xGoalRaw;
    private State100 m_yGoalRaw;
    private State100 m_thetaGoalRaw;

    public DriveWithProfile2(
            SupplierLogger parent,
            Supplier<Optional<Pose2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicDriveController100 controller,
            SwerveKinodynamics limits) {
        super(parent);
        m_fieldRelativeGoal = fieldRelativeGoal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_limits = limits;
        xProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2(),
                0.01);
        yProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2(),
                0.01);
        thetaProfile = new TrapezoidProfile100(
                m_limits.getMaxAngleSpeedRad_S(),
                m_limits.getMaxAngleAccelRad_S2(),
                0.01);
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
        Rotation2d currentRotation = m_swerve.getState().pose().getRotation();
        // take the short path
        double measurement = currentRotation.getRadians();
        Optional<Pose2d> opt = m_fieldRelativeGoal.get();
        if (opt.isEmpty()) {
            Util.warn("DriveWithProfile2: no goal!");
            return;
        }
        Pose2d fieldRelativeGoal = opt.get();

        Rotation2d bearing = new Rotation2d(
                Math100.getMinDistance(measurement, fieldRelativeGoal.getRotation().getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, thetaSetpoint.x()),
                thetaSetpoint.v());

        m_thetaGoalRaw = new State100(bearing.getRadians(), 0);
        m_xGoalRaw = new State100(fieldRelativeGoal.getX(), 0, 0);
        xSetpoint = xProfile.calculate(0.02, xSetpoint, m_xGoalRaw);

        m_yGoalRaw = new State100(fieldRelativeGoal.getY(), 0, 0);
        ySetpoint = yProfile.calculate(0.02, ySetpoint, m_yGoalRaw);

        thetaSetpoint = thetaProfile.calculate(0.02, thetaSetpoint, m_thetaGoalRaw);
        SwerveState goalState = new SwerveState(xSetpoint, ySetpoint, thetaSetpoint);
        FieldRelativeVelocity goal = m_controller.calculate(m_swerve.getState(), goalState);
        m_swerve.driveInFieldCoords(goal, 0.02);
    }

    @Override
    public boolean isFinished() {
        double xError = m_xGoalRaw.x() - m_swerve.getState().x().x();
        double yError = m_yGoalRaw.x() - m_swerve.getState().y().x();
        double thetaError = m_thetaGoalRaw.x() - m_swerve.getState().theta().x();
        return Math.abs(xError) < kTolerance
                && Math.abs(yError) < kTolerance
                && Math.abs(thetaError) < kTolerance;
    }

    @Override
    public void end100(boolean interrupted) {
        m_swerve.stop();
    }

}

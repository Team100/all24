package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.State100;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 * 
 * Sanjans version
 */
public class DriveWithProfile2 extends Command implements Glassy {
    public static double kRotationToleranceRad = Math.PI / 32;
    public static double kTranslationalToleranceM = 0.01;
    public static double kRotationToleranceRad_S = Math.PI / 64;
    public static double kTranslationalToleranceM_S = 0.01;

    private final Supplier<Optional<Pose2d>> m_fieldRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final SwerveKinodynamics m_limits;
    private TrapezoidProfile100 xProfile;
    private TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;
    private final double dt = 0.02;
    private double sx = 1;
    private double sy = 1;
    private boolean wheelsAtCorrectPos;

    private State100 xSetpoint;
    private State100 ySetpoint;
    private State100 thetaSetpoint;

    private State100 m_xGoalRaw;
    private State100 m_yGoalRaw;
    private State100 m_thetaGoalRaw;

    public DriveWithProfile2(
            Supplier<Optional<Pose2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics limits) {
        m_fieldRelativeGoal = fieldRelativeGoal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_limits = limits;
        xProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2() /2,
                kTranslationalToleranceM);
        yProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2()/2,
                kTranslationalToleranceM);
        thetaProfile = new TrapezoidProfile100(
                m_limits.getMaxAngleSpeedRad_S(),
                m_limits.getMaxAngleAccelRad_S2()/2,
                kRotationToleranceRad);
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        wheelsAtCorrectPos = false;

        xSetpoint = m_swerve.getState().x();
        ySetpoint = m_swerve.getState().y();
        thetaSetpoint = m_swerve.getState().theta();

        Optional<Pose2d> opt = m_fieldRelativeGoal.get();
        if (opt.isEmpty()) {
            return;
        }
        Pose2d fieldRelativeGoal = opt.get();
        
        m_xGoalRaw = new State100(fieldRelativeGoal.getX(), 0, 0);
        m_yGoalRaw = new State100(fieldRelativeGoal.getY(), 0, 0);

        double tx = xProfile.calculateWithETA(dt, xSetpoint, m_xGoalRaw).etaS();
        double ty = yProfile.calculateWithETA(dt, ySetpoint, m_yGoalRaw).etaS();
        double slowETA = Math.max(tx, ty);

        sx = TrapezoidProfile100.solveForSlowerETA(
                m_limits.getMaxDriveVelocityM_S(), m_limits.getMaxDriveAccelerationM_S2()/2, kTranslationalToleranceM, dt,
                xSetpoint, m_xGoalRaw, slowETA, kTranslationalToleranceM_S);
        sy = TrapezoidProfile100.solveForSlowerETA(
                m_limits.getMaxDriveVelocityM_S(), m_limits.getMaxDriveAccelerationM_S2()/2, kTranslationalToleranceM, dt,
                ySetpoint, m_yGoalRaw, slowETA, kTranslationalToleranceM_S);

        xProfile = xProfile.scale(sx);
        yProfile = yProfile.scale(sy);
    }

    @Override
    public void execute() {
        
        Rotation2d currentRotation = m_swerve.getPose().getRotation();
        // take the short path
        double measurement = currentRotation.getRadians();
        Optional<Pose2d> opt = m_fieldRelativeGoal.get();
        if (opt.isEmpty()) {
            return;
        }
        Pose2d fieldRelativeGoal = opt.get();

        Rotation2d bearing = new Rotation2d(
                Math100.getMinDistance(measurement, fieldRelativeGoal.getRotation().getRadians()));

        if (!wheelsAtCorrectPos) {
            if (atRest()) {
                m_thetaGoalRaw = new State100(bearing.getRadians(), 0);
                m_xGoalRaw = new State100(fieldRelativeGoal.getX(), 0, 0);
                State100 xs = xProfile.calculate(TimedRobot100.LOOP_PERIOD_S, xSetpoint, m_xGoalRaw);

                m_yGoalRaw = new State100(fieldRelativeGoal.getY(), 0, 0);
                State100 ys = yProfile.calculate(TimedRobot100.LOOP_PERIOD_S, ySetpoint, m_yGoalRaw);

                State100 thetas = thetaProfile.calculate(TimedRobot100.LOOP_PERIOD_S, thetaSetpoint, m_thetaGoalRaw);
                SwerveState goalState = new SwerveState(xs, ys, thetas);
                FieldRelativeVelocity goal = m_controller.calculate(m_swerve.getState(), goalState);
                wheelsAtCorrectPos = m_swerve.steerAtRest(goal);
                return;
            } else {
                wheelsAtCorrectPos = true;
            }
        }
        // make sure the setpoint uses the modulus close to the measurement.
        thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, thetaSetpoint.x()),
                thetaSetpoint.v());

        m_thetaGoalRaw = new State100(bearing.getRadians(), 0);
        m_xGoalRaw = new State100(fieldRelativeGoal.getX(), 0, 0);
        xSetpoint = xProfile.calculate(TimedRobot100.LOOP_PERIOD_S, xSetpoint, m_xGoalRaw);

        m_yGoalRaw = new State100(fieldRelativeGoal.getY(), 0, 0);
        ySetpoint = yProfile.calculate(TimedRobot100.LOOP_PERIOD_S, ySetpoint, m_yGoalRaw);

        thetaSetpoint = thetaProfile.calculate(TimedRobot100.LOOP_PERIOD_S, thetaSetpoint, m_thetaGoalRaw);
        SwerveState goalState = new SwerveState(xSetpoint, ySetpoint, thetaSetpoint);
        FieldRelativeVelocity goal = m_controller.calculate(m_swerve.getState(), goalState);
        m_swerve.driveInFieldCoords(goal);
    }

    @Override
    public boolean isFinished() {
        if (!m_fieldRelativeGoal.get().isPresent()) return true;
        double xError = m_xGoalRaw.x() - m_swerve.getState().x().x();
        double yError = m_yGoalRaw.x() - m_swerve.getState().y().x();
        double thetaError = m_thetaGoalRaw.x() - m_swerve.getState().theta().x();
        return Math.abs(xError) < kTranslationalToleranceM
                && Math.abs(yError) < kTranslationalToleranceM
                && Math.abs(thetaError) < kRotationToleranceRad
                && atRest();
    }

    public boolean atRest() {
        return Math.abs(m_swerve.getState().x().v()) < kTranslationalToleranceM_S
        && Math.abs(m_swerve.getState().y().v()) < kTranslationalToleranceM_S
        && Math.abs(m_swerve.getState().theta().v()) < kRotationToleranceRad_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        xProfile = xProfile.scale(1/sx);
        yProfile = yProfile.scale(1/sy);
    }

}

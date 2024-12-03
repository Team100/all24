package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
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

    private final Supplier<Optional<Pose2d>> m_fieldRelativeGoalSupplier;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final SwerveKinodynamics m_limits;
    private TrapezoidProfile100 xProfile;
    private TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;
    private final double dt = 0.02;
    private double sx = 1;
    private double sy = 1;

    private Pose2d m_fieldRelativeGoal = null;
    
    private final FieldLogger.Log m_field_log;

    private Control100 xSetpoint;
    private Control100 ySetpoint;
    private Control100 thetaSetpoint;

    private Model100 m_xGoalRaw;
    private Model100 m_yGoalRaw;
    private Model100 m_thetaGoalRaw;

    public DriveWithProfile2(
            FieldLogger.Log fieldLogger,
            Supplier<Optional<Pose2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics limits) {
        m_field_log = fieldLogger;
        m_fieldRelativeGoalSupplier = fieldRelativeGoal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_limits = limits;
        xProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2()/2,
                kTranslationalToleranceM);
        yProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2()/2,
                kTranslationalToleranceM);
        thetaProfile = new TrapezoidProfile100(
                m_limits.getMaxAngleSpeedRad_S(),
                m_limits.getMaxAngleAccelRad_S2()/50,
                kRotationToleranceRad);
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        xSetpoint = m_swerve.getState().x().control();
        ySetpoint = m_swerve.getState().y().control();
        thetaSetpoint = m_swerve.getState().theta().control();

        Optional<Pose2d> opt = m_fieldRelativeGoalSupplier.get();
        if (opt.isEmpty()) {
            if (m_fieldRelativeGoal == null) {
                return;
            }
        } else {
            m_fieldRelativeGoal = opt.get();
        }
        
        m_xGoalRaw = new Model100(m_fieldRelativeGoal.getX(), 0);
        m_yGoalRaw = new Model100(m_fieldRelativeGoal.getY(), 0);

        double tx = xProfile.calculateWithETA(dt, xSetpoint.model(), m_xGoalRaw).etaS();
        double ty = yProfile.calculateWithETA(dt, ySetpoint.model(), m_yGoalRaw).etaS();
        double slowETA = Math.max(tx, ty);

        sx = TrapezoidProfile100.solveForSlowerETA(
                m_limits.getMaxDriveVelocityM_S(), m_limits.getMaxDriveAccelerationM_S2()/2, kTranslationalToleranceM, dt,
                xSetpoint.model(), m_xGoalRaw, slowETA, kTranslationalToleranceM_S);
        sy = TrapezoidProfile100.solveForSlowerETA(
                m_limits.getMaxDriveVelocityM_S(), m_limits.getMaxDriveAccelerationM_S2()/2, kTranslationalToleranceM, dt,
                ySetpoint.model(), m_yGoalRaw, slowETA, kTranslationalToleranceM_S);

        xProfile = xProfile.scale(sx);
        yProfile = yProfile.scale(sy);
    }

    @Override
    public void execute() {
        
        Rotation2d currentRotation = m_swerve.getPose().getRotation();
        // take the short path
        double measurement = currentRotation.getRadians();
        Optional<Pose2d> opt = m_fieldRelativeGoalSupplier.get();
        if (opt.isEmpty()) {
            if (m_fieldRelativeGoal == null) {
                return;
            }
        } else {
            m_fieldRelativeGoal = opt.get();
        }
        Rotation2d rotation = new Rotation2d(
                Math100.getMinDistance(measurement, m_fieldRelativeGoal.getRotation().getRadians()));
        // make sure the setpoint uses the modulus close to the measurement.
        thetaSetpoint = new Control100(
                Math100.getMinDistance(measurement, thetaSetpoint.x()),
                thetaSetpoint.v());

        m_thetaGoalRaw = new Model100(rotation.getRadians(), 0);
        m_xGoalRaw = new Model100(m_fieldRelativeGoal.getX(), 0);
        xSetpoint = xProfile.calculate(TimedRobot100.LOOP_PERIOD_S, xSetpoint.model(), m_xGoalRaw);

        m_yGoalRaw = new Model100(m_fieldRelativeGoal.getY(), 0);
        ySetpoint = yProfile.calculate(TimedRobot100.LOOP_PERIOD_S, ySetpoint.model(), m_yGoalRaw);
        m_field_log.m_log_target.log(() -> new double[] {
            m_fieldRelativeGoal.getX(),
            m_fieldRelativeGoal.getY(),
            m_fieldRelativeGoal.getRotation().getRadians() });

        thetaSetpoint = thetaProfile.calculate(TimedRobot100.LOOP_PERIOD_S, thetaSetpoint.model(), m_thetaGoalRaw);
        SwerveModel goalState = new SwerveModel(xSetpoint.model(), ySetpoint.model(), thetaSetpoint.model());
        FieldRelativeVelocity goal = m_controller.calculate(m_swerve.getState(), goalState);
        m_swerve.driveInFieldCoords(goal);
    }

    @Override
    public boolean isFinished() {
        if (!m_fieldRelativeGoalSupplier.get().isPresent() && m_fieldRelativeGoal == null) {
            return true;
        }
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

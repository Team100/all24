package org.team100.frc2024.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2024.motion.intake.Intake;
import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Creates a profile to the translation of a note and follows it.
 * 
 * If the goal supplier runs empty, this remembers the previous goal for 1 sec,
 * and then gives up.
 */
public class DriveWithProfileNote extends Command implements Glassy {

    private static final double kRotationToleranceRad = Math.PI / 32;
    private static final double kTranslationalToleranceM = 0.01;
    private static final double kRotationToleranceRad_S = Math.PI / 64;
    private static final double kTranslationalToleranceM_S = 0.01;

    private final FieldLogger.Log m_field_log;
    private final Intake m_intake;
    private final Supplier<Optional<Translation2d>> m_fieldRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final SwerveKinodynamics m_limits;
    private final TrapezoidProfile100 xProfile;
    private final TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;

    // LOGGERS
    private final BooleanLogger m_log_note_detected;

    private Translation2d m_previousGoal;
    private Control100 m_xSetpoint;
    private Control100 m_ySetpoint;
    private Control100 m_thetaSetpoint;
    private int m_count;

    private Model100 m_xGoalRaw;
    private Model100 m_yGoalRaw;
    private Model100 m_thetaGoalRaw;

    public DriveWithProfileNote(
            FieldLogger.Log fieldLogger,
            LoggerFactory parent,
            Intake intake,
            Supplier<Optional<Translation2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics limits) {
        m_field_log = fieldLogger;
        LoggerFactory child = parent.child(this);
        m_log_note_detected = child.booleanLogger(Level.TRACE, "Note detected");

        m_intake = intake;
        m_fieldRelativeGoal = fieldRelativeGoal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_limits = limits;

        xProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2() / 2,
                0.01);
        yProfile = new TrapezoidProfile100(
                m_limits.getMaxDriveVelocityM_S(),
                m_limits.getMaxDriveAccelerationM_S2() / 2,
                0.01);
        thetaProfile = new TrapezoidProfile100(
                m_limits.getMaxAngleSpeedRad_S(),
                m_limits.getMaxAngleAccelRad_S2() / 4,
                0.01);

        m_previousGoal = null;
        m_count = 0;

        addRequirements(m_swerve, m_intake);
    }

    @Override
    public void initialize() {
        m_xSetpoint = m_swerve.getState().x().control();
        m_ySetpoint = m_swerve.getState().y().control();
        m_thetaSetpoint = m_swerve.getState().theta().control();
    }

    /**
     * Returns the current goal, or the previous one if the current one is newly
     * empty.
     */
    private Optional<Translation2d> getGoal() {
        Optional<Translation2d> optGoal = m_fieldRelativeGoal.get();
        m_log_note_detected.log(optGoal::isPresent);
        if (optGoal.isPresent()) {
            // Supplier is ok, use this goal and reset the history mechanism.
            m_previousGoal = optGoal.get();
            m_count = 0;
            return optGoal;
        }
        if (m_count > 50) {
            // Supplier is empty and timer has expired.
            return Optional.empty();
        }
        if (m_previousGoal == null) {
            // Nothing to fall back to.
            return Optional.empty();
        }
        m_count++;
        return Optional.of(m_previousGoal);
    }

    @Override
    public void execute() {
        // intake the whole time
        m_intake.intakeSmart();

        Optional<Translation2d> optGoal = getGoal();
        if (optGoal.isEmpty()) {
            // No current goal, timer expired, or no past goal.
            return;
        }

        Translation2d goal = optGoal.get();

        m_thetaGoalRaw = getThetaGoalState(m_swerve.getPose(), goal);
        m_xGoalRaw = new Model100(goal.getX(), 0);
        m_yGoalRaw = new Model100(goal.getY(), 0);

        m_xSetpoint = xProfile.calculate(TimedRobot100.LOOP_PERIOD_S, m_xSetpoint.model(), m_xGoalRaw);
        m_ySetpoint = yProfile.calculate(TimedRobot100.LOOP_PERIOD_S, m_ySetpoint.model(), m_yGoalRaw);
        // make sure the setpoint uses the modulus close to the measurement.
        final double thetaMeasurement = m_swerve.getPose().getRotation().getRadians();
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(thetaMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());
        m_thetaSetpoint = thetaProfile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint.model(), m_thetaGoalRaw);

        SwerveModel measurement = m_swerve.getState();
        SwerveModel setpoint = new SwerveModel(m_xSetpoint.model(), m_ySetpoint.model(), m_thetaSetpoint.model());
        FieldRelativeVelocity output = m_controller.calculate(measurement, setpoint);

        m_swerve.driveInFieldCoords(output);

        m_field_log.m_log_target.log(() -> new double[] { goal.getX(), goal.getY(), 0 });
    }

    private static Model100 getThetaGoalState(Pose2d pose, Translation2d goal) {
        // take the short path
        final double measurementRad = pose.getRotation().getRadians();
        final double goalRad = getThetaGoalRad(goal, pose);
        return new Model100(Math100.getMinDistance(measurementRad, goalRad), 0);
    }

    private static double getThetaGoalRad(Translation2d goal, Pose2d pose) {
        if (Experiments.instance.enabled(Experiment.DriveToNoteWithRotation)) {
            // face the rear of the robot towards the goal.
            Translation2d toGoal = goal.minus(pose.getTranslation());
            return toGoal.getAngle().getRadians() + Math.PI;
        } else {
            // leave the rotation alone
            return pose.getRotation().getRadians();
        }
    }

    @Override
    public boolean isFinished() {
        Model100 x = m_swerve.getState().x();
        double xError = m_xGoalRaw.x() - x.x();
        Model100 y = m_swerve.getState().y();
        double yError = m_yGoalRaw.x() - y.x();
        Model100 theta = m_swerve.getState().theta();
        double thetaError = m_thetaGoalRaw.x() - theta.x();
        return Math.abs(xError) < kTranslationalToleranceM
                && Math.abs(yError) < kTranslationalToleranceM
                && Math.abs(thetaError) < kRotationToleranceRad
                && Math.abs(x.v()) < kTranslationalToleranceM_S
                && Math.abs(y.v()) < kTranslationalToleranceM_S
                && Math.abs(theta.v()) < kRotationToleranceRad_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}

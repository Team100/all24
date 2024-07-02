package org.team100.frc2024.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2024.motion.intake.Intake;
import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.controller.State100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Creates a profile to the translation of a note and follows it
 */
public class DriveWithProfileNote extends Command100 {
    private final Telemetry t = Telemetry.get();

    private final Intake m_intake;

    private final Supplier<Optional<Translation2d>> m_fieldRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicDriveController100 m_controller;
    private final SwerveKinodynamics m_limits;
    private final TrapezoidProfile100 xProfile;
    private final TrapezoidProfile100 yProfile;
    private final TrapezoidProfile100 thetaProfile;
    private Optional<Translation2d> previousGoal;
    private State100 xSetpoint;
    private State100 ySetpoint;
    private State100 thetaSetpoint;
    private int count;

    public DriveWithProfileNote(
            Intake intake,
            Supplier<Optional<Translation2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicDriveController100 controller,
            SwerveKinodynamics limits) {
        previousGoal = null;
        count = 0;
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
        addRequirements(m_swerve, m_intake);
    }

    @Override
    public void initialize100() {
        xSetpoint = m_swerve.getState().x();
        ySetpoint = m_swerve.getState().y();
        thetaSetpoint = m_swerve.getState().theta();
    }

    @Override
    public void execute100(double dt) {
        // intake the whole time
        m_intake.intakeSmart();
        Optional<Translation2d> optGoal = m_fieldRelativeGoal.get();
        if (optGoal.isEmpty()) {
            if (previousGoal == null) {
                m_swerve.setChassisSpeeds(new ChassisSpeeds(), dt);
                t.log(Level.DEBUG, m_name, "Note detected", false);
                return;
            }
            optGoal = previousGoal;
            count++;
            if (count == 50) {
                return;
            }
        } else {
            count = 0;
        }
        Translation2d goal = optGoal.get();

        t.log(Level.DEBUG, m_name, "Note detected", true);
        Rotation2d rotationGoal;
        if (Experiments.instance.enabled(Experiment.DriveToNoteWithRotation)) {
            rotationGoal = new Rotation2d(
                    goal.minus(m_swerve.getState().pose().getTranslation()).getAngle().getRadians() + Math.PI);
        } else {
            rotationGoal = m_swerve.getState().pose().getRotation();
        }
        // take the short path
        double measurement = m_swerve.getState().pose().getRotation().getRadians();
        rotationGoal = new Rotation2d(
                Math100.getMinDistance(measurement, rotationGoal.getRadians()));

        // make sure the setpoint uses the modulus close to the measurement.
        thetaSetpoint = new State100(
                Math100.getMinDistance(measurement, thetaSetpoint.x()),
                thetaSetpoint.v());

        State100 thetaGoal = new State100(rotationGoal.getRadians(), 0);
        State100 xGoalRaw = new State100(goal.getX(), 0, 0);
        State100 yGoalRaw = new State100(goal.getY(), 0, 0);

        xSetpoint = xProfile.calculate(dt, xSetpoint, xGoalRaw);
        ySetpoint = yProfile.calculate(dt, ySetpoint, yGoalRaw);
        thetaSetpoint = thetaProfile.calculate(dt, thetaSetpoint, thetaGoal);

        SwerveState goalState = new SwerveState(xSetpoint, ySetpoint, thetaSetpoint);
        FieldRelativeVelocity twistGoal = m_controller.calculate(m_swerve.getState(), goalState);

        t.log(Level.DEBUG, "field", "target", new double[] {
                goal.getX(),
                goal.getY(),
                0 });
        m_swerve.driveInFieldCoords(twistGoal, dt);

    }

    @Override
    public void end100(boolean interrupted) {
    }
}

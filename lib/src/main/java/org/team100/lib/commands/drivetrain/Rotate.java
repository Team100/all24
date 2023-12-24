package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Rotate in place to the specified angle.
 * 
 * Uses a profile with the holonomic drive controller.
 */
public class Rotate extends Command {
    private static final double kXToleranceRad = 0.003;
    private static final  double kVToleranceRad_S = 0.003;

    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final HeadingInterface m_heading;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Timer m_timer;
    private final TrapezoidProfile.State m_goalState;
    private final HolonomicDriveController3 m_controller;
    private double prevTime;

    TrapezoidProfile m_profile; // set in initialize(), package private for testing
    TrapezoidProfile.State refTheta; // updated in execute(), package private for testing.

    public Rotate(
            SwerveDriveSubsystemInterface drivetrain,
            HeadingInterface heading,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        m_robotDrive = drivetrain;
        // since we specify a different tolerance, use a new controller.
        m_controller = HolonomicDriveController3.withTolerance(0.1, 0.1, kXToleranceRad, kVToleranceRad_S);
        m_heading = heading;
        m_swerveKinodynamics = swerveKinodynamics;
        m_timer = new Timer();
        m_goalState = new TrapezoidProfile.State(targetAngleRadians, 0);
        refTheta = new TrapezoidProfile.State(0, 0);

        if (drivetrain.get() != null)
            addRequirements(drivetrain.get());
    }

    @Override
    public void initialize() {
        m_controller.reset();
        ChassisSpeeds initialSpeeds = m_robotDrive.speeds();
        refTheta = new TrapezoidProfile.State(
            m_robotDrive.getPose().getRotation().getRadians(),
                initialSpeeds.omegaRadiansPerSecond);
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(
                m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleAccelRad_S2());
        m_profile = new TrapezoidProfile(c);
        m_timer.restart();
        prevTime = 0;
    }

    @Override
    public void execute() {
        double now = m_timer.get();
        // reference
        double dt = now - prevTime;
        refTheta = m_profile.calculate(dt, m_goalState, refTheta);
        prevTime = now;
        // measurement
        Pose2d currentPose = m_robotDrive.getPose();

        SwerveState reference = new SwerveState(
                new State100(currentPose.getX(), 0, 0), // stationary at current pose
                new State100(currentPose.getY(), 0, 0),
                new State100(refTheta.position, refTheta.velocity, 0)); // TODO: accel

        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
        m_robotDrive.driveInFieldCoords(fieldRelativeTarget);

        double headingMeasurement = currentPose.getRotation().getRadians();
        // note the use of Heading here.
        // TODO: extend the pose estimator to include rate.
        double headingRate = m_heading.getHeadingRateNWU();

        // log what we did
        t.log(Level.DEBUG, "/rotate/errorX", refTheta.position - headingMeasurement);
        t.log(Level.DEBUG, "/rotate/errorV", refTheta.velocity - headingRate);
        t.log(Level.DEBUG, "/rotate/measurementX", headingMeasurement);
        t.log(Level.DEBUG, "/rotate/measurementV", headingRate);
        t.log(Level.DEBUG, "/rotate/refX", refTheta.position);
        t.log(Level.DEBUG, "/rotate/refV", refTheta.velocity);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > m_profile.totalTime() && m_controller.atReference();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }
}

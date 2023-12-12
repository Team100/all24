package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Rotate in place to the specified angle.
 * 
 * Uses a MotionProfile with the holonomic drive controller.
 */
public class Rotate extends Command {
    private static final double kXToleranceRad = 0.003;
    private static final  double kVToleranceRad_S = 0.003;

    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final HeadingInterface m_heading;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final MotionState m_goalState;
    private final HolonomicDriveController3 m_controller;

    MotionProfile m_profile; // set in initialize(), package private for testing
    MotionState refTheta; // updated in execute(), package private for testing.

    public Rotate(
            SwerveDriveSubsystemInterface drivetrain,
            HeadingInterface heading,
            SpeedLimits speedLimits,
            double targetAngleRadians) {
        m_robotDrive = drivetrain;
        // since we specify a different tolerance, use a new controller.
        m_controller = HolonomicDriveController3.withTolerance(0.1, 0.1, kXToleranceRad, kVToleranceRad_S);
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_timer = new Timer();
        m_goalState = new MotionState(targetAngleRadians, 0);
        refTheta = new MotionState(0, 0);

        if (drivetrain.get() != null)
            addRequirements(drivetrain.get());
    }

    @Override
    public void initialize() {
        m_controller.reset();
        ChassisSpeeds initialSpeeds = m_robotDrive.speeds();
        MotionState start = new MotionState(m_robotDrive.getPose().getRotation().getRadians(),
                initialSpeeds.omegaRadiansPerSecond);
        m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                m_goalState,
                m_speedLimits.angleSpeedRad_S,
                m_speedLimits.angleAccelRad_S2,
                m_speedLimits.angleJerkRad_S3);

        m_timer.restart();
    }

    @Override
    public void execute() {
        // reference
        refTheta = m_profile.get(m_timer.get());
        // measurement
        Pose2d currentPose = m_robotDrive.getPose();

        SwerveState reference = new SwerveState(
                new State100(currentPose.getX(), 0, 0), // stationary at current pose
                new State100(currentPose.getY(), 0, 0),
                new State100(refTheta.getX(), refTheta.getV(), refTheta.getA()));

        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
        m_robotDrive.driveInFieldCoords(fieldRelativeTarget);

        double headingMeasurement = currentPose.getRotation().getRadians();
        // note the use of Heading here.
        // TODO: extend the pose estimator to include rate.
        double headingRate = m_heading.getHeadingRateNWU();

        // log what we did
        t.log(Level.DEBUG, "/rotate/errorX", refTheta.getX() - headingMeasurement);
        t.log(Level.DEBUG, "/rotate/errorV", refTheta.getV() - headingRate);
        t.log(Level.DEBUG, "/rotate/measurementX", headingMeasurement);
        t.log(Level.DEBUG, "/rotate/measurementV", headingRate);
        t.log(Level.DEBUG, "/rotate/refX", refTheta.getX());
        t.log(Level.DEBUG, "/rotate/refV", refTheta.getV());
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > m_profile.duration() && m_controller.atReference();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }
}

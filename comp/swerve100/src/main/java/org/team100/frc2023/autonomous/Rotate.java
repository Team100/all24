package org.team100.frc2023.autonomous;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Uses a timed profile, which avoids the bad ProfiledPidController behavior of
 * staying behind. Also demonstrates Roadrunner MotionProfiles.
 */
public class Rotate extends Command {
    public static class Config {
        public double xToleranceRad = 0.003;
        public double vToleranceRad_S = 0.003;
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final HeadingInterface m_heading;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final MotionState m_goalState;
    MotionProfile m_profile; // set in initialize(), package private for testing
    MotionState refTheta; // updated in execute(), package private for testing.
    private double headingMeasurement; // updated in execute()
    private double headingRate;

    public Rotate(
            SwerveDriveSubsystemInterface drivetrain,
            HeadingInterface heading,
            SpeedLimits speedLimits,
            Timer timer,
            double targetAngleRadians) {
        m_robotDrive = drivetrain;
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_timer = timer;
        m_goalState = new MotionState(targetAngleRadians, 0);
        refTheta = new MotionState(0, 0);
        if (drivetrain.get() != null)
            addRequirements(drivetrain.get());
    }

    @Override
    public void initialize() {
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

        // tell the drivetrain the state we want
        m_robotDrive.setDesiredState(
                new SwerveState(
                        new State100(currentPose.getX(), 0, 0), // stationary at current pose
                        new State100(currentPose.getY(), 0, 0),
                        new State100(refTheta.getX(), refTheta.getV(), refTheta.getA())));

        // update measurement (this is for isFinished())
        headingMeasurement = currentPose.getRotation().getRadians();
        // note the use of Heading here.
        // TODO: extend the pose estimator to include rate.
        headingRate = m_heading.getHeadingRateNWU();

        // log what we did
        t.log("/rotate/errorX", xErrorRad());
        t.log("/rotate/errorV", vErrorRad_S());
        t.log("/rotate/measurementX", headingMeasurement);
        t.log("/rotate/measurementV", headingRate);
        t.log("/rotate/refX", refTheta.getX());
        t.log("/rotate/refV", refTheta.getV());
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > m_profile.duration() && atSetpoint();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.truncate();
    }

    ////////////////////////////////////////////////////////////////////////////

    private double xErrorRad() {
        return refTheta.getX() - headingMeasurement;
    }

    private double vErrorRad_S() {
        return refTheta.getV() - headingRate;
    }

    private boolean atSetpoint() {
        return Math.abs(xErrorRad()) < m_config.xToleranceRad && Math.abs(vErrorRad_S()) < m_config.vToleranceRad_S;
    }
}

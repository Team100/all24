package org.team100.frc2023.autonomous;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Profiled small movement in Y, holding X constant and theta at 180 deg.
 */
public class MoveConeWidth extends Command {
    public static class Config {
        public double goalRotation = Math.PI;
        public double widthM = 0.155;
        public double speedM_S = 0.25;
        public double accelM_S2 = 0.25;
        public double jerkM_S3 = 0.25;
        public double xToleranceM = 0.01;
        public double vToleranceM_S = 0.01;
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final boolean m_left;

    private MotionProfile m_profile; // created in initialize(), sampled in execute()
    private MotionState m_ref;
    private double m_measurement;
    private double m_measurementRate;

    public MoveConeWidth(SwerveDriveSubsystem robotDrive,
            SpeedLimits speedLimits,
            Timer timer,
            boolean left) {
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        m_timer = timer;
        m_left = left;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds initialSpeeds = m_robotDrive.speeds();
        MotionState start = new MotionState(currentPose.getY(), initialSpeeds.vyMetersPerSecond);

        double m_coneWidthM = m_config.widthM * (m_left ? 1.0 : -1.0);
        MotionState m_goal = new MotionState(currentPose.getY() + m_coneWidthM, 0); // goal is stationary

        m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                m_goal,
                Math.min(m_config.speedM_S, m_speedLimits.speedM_S),
                Math.min(m_config.accelM_S2, m_speedLimits.accelM_S2),
                Math.min(m_config.jerkM_S3, m_speedLimits.jerkM_S3));
                
        m_timer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_robotDrive.getPose();
        m_ref = m_profile.get(m_timer.get());

        m_measurement = currentPose.getY();
        ChassisSpeeds speeds = m_robotDrive.speeds();
        m_measurementRate = speeds.vyMetersPerSecond;

        m_robotDrive.setDesiredState(
                new SwerveState(
                        new State100(currentPose.getX(), 0, 0), // hold x
                        new State100(m_ref.getX(), m_ref.getV(), m_ref.getA()), // track y
                        new State100(m_config.goalRotation, 0, 0)) // hold theta
        );
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > m_profile.duration() && atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }

    /////////////////////////////////////////////////////////

    private double xErrorRad() {
        return m_ref.getX() - m_measurement;
    }

    private double vErrorRad_S() {
        return m_ref.getV() - m_measurementRate;
    }

    private boolean atSetpoint() {
        return Math.abs(xErrorRad()) < m_config.xToleranceM && Math.abs(vErrorRad_S()) < m_config.vToleranceM_S;
    }
}

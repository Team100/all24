package org.team100.frc2023.commands;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithLQR extends Command {
    public static class Config {
        public double xToleranceM = 0.01;
        public double vToleranceM_S = 0.01;
        public double speedM_S = 5.0;
        public double accelM_S2 = 5.0;
        public double jerkM_S3 = 5.0;
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;

    // TrapezoidProfile.State goal;

    // boolean done = false;

    // private final TrapezoidProfile.Constraints m_constraints = new
    // TrapezoidProfile.Constraints(
    // 5,
    // 5);
    // private TrapezoidProfile.State m_lastProfiledReference = new
    // TrapezoidProfile.State();

    private final LinearSystem<N2, N1, N1> m_drivePlant = LinearSystemId.identifyPositionSystem(.5, 0.025);

    private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            m_drivePlant,
            VecBuilder.fill(0.015, 0.17), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

    private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
            m_drivePlant,
            VecBuilder.fill(0.5, 10.0), // qelms.
            VecBuilder.fill(12), // relms. Control effort (voltage) tolerance. Decrease this to more
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

    private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_drivePlant, m_controller, m_observer,
            12.0, 0.020);

    private MotionState m_goal;
    private MotionProfile m_profile;
    private MotionState m_ref;

    public DriveWithLQR(SwerveDriveSubsystem robotDrive, SpeedLimits speedLimits, Timer timer) {
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        m_timer = timer;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_robotDrive.getPose();

        m_loop.reset(VecBuilder.fill(currentPose.getX(), 0));

        double currentX = currentPose.getX();
        double goalX = currentX + 1;

        MotionState start = new MotionState(currentX, 0);
        m_goal = new MotionState(goalX, 0);

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
        m_ref = m_profile.get(m_timer.get());

        m_loop.setNextR(m_ref.getX(), m_ref.getV());

        m_loop.correct(VecBuilder.fill(m_robotDrive.getPose().getX()));

        m_loop.predict(0.020);

        double u = m_loop.getU(0);

        Twist2d fieldRelative = new Twist2d(u, 0, 0);

        Pose2d currentPose = m_robotDrive.getPose();
        SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, fieldRelative);
        m_robotDrive.setDesiredState(manualState);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_ref.getX() - m_goal.getX()) < m_config.xToleranceM
                && Math.abs(m_ref.getV() - m_goal.getV()) < m_config.vToleranceM_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}

package org.team100.lib.autonomous;

import java.util.List;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This is a simpler way to drive to a waypoint. It's just like
 * SwerveControllerCommand except that it generates the trajectory at the time
 * the command is scheduled, so it can capture the current robot location at
 * that instant. It runs forever, so it expects to be scheduled via
 * Trigger.whileTrue().
 */

public class DriveToWaypoint3 extends Command {
    public static class Config {

        public TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(8, 12);
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final SwerveDriveKinematics m_kinematics;
    private final Timer m_timer;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;

    private final PIDController xController;
    private final PIDController yController;
    // private final HolonomicLQR m_controller;
    private final HolonomicDriveController3 m_controller;

    // private GoalOffset previousOffset;
    private Trajectory m_trajectory;
    private boolean isFinished = false;

    // private final LQRManager xManager;
    // private final LQRManager yManager;

    // private final Manipulator m_manipulator;

    // private Translation2d globalGoalTranslation;

    int count = 0;

    Matrix<N2, N2> matrix = new Matrix<>(Nat.N2(), Nat.N2());

    // Matrix<N2, N2> matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    // matrixA.fill(0, 0, 0, 1, 0, 0)

    // LinearPlantInversionFeedforward<N2, N2, N1> feedforward = new
    // LinearPlantInversionFeedforward<>(), count)

    // private State desiredStateGlobal;

    public DriveToWaypoint3(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            SwerveDriveKinematics kinematics) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_kinematics = kinematics;

        m_timer = new Timer();

        m_rotationController = new ProfiledPIDController(6.5, 0, 1, m_config.rotationConstraints);
        m_rotationController.setTolerance(Math.PI / 180);

        xController = new PIDController(2, 0, 0);
        xController.setIntegratorRange(-0.3, 0.3);
        xController.setTolerance(0.00000001);

        yController = new PIDController(2, 0, 0);
        yController.setIntegratorRange(-0.3, 0.3);
        yController.setTolerance(0.00000001);

        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
                5,
                5);

        LinearSystem<N2, N1, N1> m_translationPlant = LinearSystemId.identifyPositionSystem(1.3, 0.06);

        KalmanFilter<N2, N1, N1> m_translationObserver = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                m_translationPlant,
                VecBuilder.fill(0.015, 0.17), // How accurate we
                // think our model is, in radians and radians/sec
                VecBuilder.fill(0.01), // How accurate we think our encoder position
                // data is. In this case we very highly trust our encoder position reading.
                0.020);

        LinearQuadraticRegulator<N2, N1, N1> m_translationController = new LinearQuadraticRegulator<>(
                m_translationPlant,
                VecBuilder.fill(0.05, 1), // qelms.
                VecBuilder.fill(20), // relms. Control effort (voltage) tolerance. Decrease this to more
                0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        // xManager = new LQRManager(m_translationPlant, m_translationObserver, m_translationController, m_constraints);
        // yManager = new LQRManager(m_translationPlant, m_translationObserver, m_translationController, m_constraints);

        // m_controller = new HolonomicDriveController2(xController, yController,
        // m_rotationController, m_gyro);
        // m_controller = new HolonomicLQR(m_swerve, xManager, yManager, m_rotationController);
        // m_controller = new HolonomicDriveController2(xController, yController);
        
        Identity identity = Identity.get();
        SpeedLimits speedLimits = new SpeedLimits(2, 2, 1, 1);

        DriveControllers controllers = new DriveControllersFactory().get(identity, speedLimits);

        m_controller = new HolonomicDriveController3(controllers);
        // m_rotationController, m_gyro);

        // globalGoalTranslation = new Translation2d();

        // m_manipulator = manipulator;

        addRequirements(m_swerve);

        translationConfig = new TrajectoryConfig(2, 2).setKinematics(kinematics);
        addRequirements(drivetrain);
    }

    private Trajectory makeTrajectory(double startVelocity) {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();

        Transform2d goalTransform = new Transform2d();

        Pose2d transformedGoal = m_goal.plus(goalTransform);

        Translation2d goalTranslation = transformedGoal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        TrajectoryConfig withStartVelocityConfig = new TrajectoryConfig(5, 2).setKinematics(m_kinematics);
        withStartVelocityConfig.setStartVelocity(startVelocity);

        try {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentTranslation, angleToGoal),
                    List.of(),
                    new Pose2d(goalTranslation, angleToGoal),
                    translationConfig);
        } catch (TrajectoryGenerationException e) {
            isFinished = true;
            return null;
        }
    }

    @Override
    public void initialize() {
        isFinished = false;
        m_timer.restart();
        count = 0;
        // m_controller.reset(m_swerve.getPose());
        // m_controller.updateProfile(m_goal.getX(), m_goal.getY(), 5, 3, 1);
        // m_controller.start();
        m_trajectory = makeTrajectory(0);

    }

    @Override
    public void execute() {
        // if (m_trajectory == null) {
        // return;
        // }
        // if (goalOffsetSupplier.get() != previousOffset) {
        // m_trajectory = makeTrajectory(goalOffsetSupplier.get(),
        // m_trajectory.sample(m_timer.get()).velocityMetersPerSecond);
        // previousOffset = goalOffsetSupplier.get();
        // m_timer.restart();
        // }
        // if (m_trajectory == null) {
        // return;
        // }
        double curTime = m_timer.get();

        if(m_trajectory != null){
            State desiredState = m_trajectory.sample(curTime);
        

        
                  
        Pose2d currentPose = m_swerve.getPose();

        // System.out.println(desiredState);

        if(currentPose.getX() > m_goal.getX() - 0.1 && currentPose.getX() < m_goal.getX() + 0.1){
            if(currentPose.getY() > m_goal.getY() - 0.1 && currentPose.getY() < m_goal.getY() + 0.1){
                // System.out.println("AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
                System.out.println(desiredState.poseMeters.getY());
                System.out.println(m_goal.getY());

                
                isFinished = true;
            } else{
                // System.out.println("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
                isFinished = false;
            }
        } else {
            isFinished = false;
            // System.out.println("BAHAHAHAHAHAHAHAHAHH");
        }

        

        // System.out.println(currentPose);
        // Twist2d fieldRelativeTarget = m_controller.calculate(
        //         currentPose,
        //         desiredState,
        //         m_goal.getRotation());
        SwerveState des = new SwerveState(
            new State100(desiredState.poseMeters.getX(), 0, 0),
            new State100(desiredState.poseMeters.getY(), 0, 0),
            new State100(0, 0, 0));
        
        
        Twist2d fieldRelativeTarget = m_controller.calculate(
                currentPose,
                des
                );

        SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, fieldRelativeTarget);
        m_swerve.setDesiredState(manualState);

        t.log("/Drive To Waypoint/Desired X", desiredState.poseMeters.getX());
        t.log("/Drive To Waypoint/Desired Y", desiredState.poseMeters.getY());
        t.log("/Drive To Waypoint/Pose X", m_swerve.getPose().getX());
        t.log("/Drive To Waypoint/Pose Y", m_swerve.getPose().getY());
        t.log("/Drive To Waypoint/Desired Rot", m_goal.getRotation().getRadians());
        t.log("/Drive To Waypoint/Rot Setpoint", m_rotationController.getSetpoint().position);
        t.log("/Drive To Waypoint/Pose Rot", m_swerve.getPose().getRotation().getRadians());
        t.log("/Drive To Waypoint/Error X", xController.getPositionError());
        t.log("/Drive To Waypoint/Error Y", yController.getPositionError());
    }
    }

    @Override
    public boolean isFinished() {
        if(isFinished){
            System.out.println("FIIIIIIIINNNNNNNNNNNNNNNNNNNNNNNn");
        }
        return isFinished; // keep trying until the button is released
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
        m_timer.stop();
        m_swerve.truncate();
    }
}

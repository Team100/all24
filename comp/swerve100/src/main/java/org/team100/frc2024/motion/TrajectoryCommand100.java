package org.team100.frc2024.motion;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follow a fixed trajectory, using the new 254-derived trajectory and follower
 * types.
 * 
 * This is an experiment.
 */
public class TrajectoryCommand100 extends Command100 {
    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveMotionController m_controller;
    private Trajectory100 m_trajectory;
    private double m_timeBuffer;
    private Pose2d m_goal;

    private Timer m_timer = new Timer();

    public TrajectoryCommand100(
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            DriveMotionController controller) {
        m_timeBuffer = 1;
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        addRequirements(m_robotDrive);
    }

    public TrajectoryCommand100(
            SwerveDriveSubsystem robotDrive,
            Trajectory100 trajectory,
            DriveMotionController controller,
            double timeBuffer) {
        m_timeBuffer = timeBuffer;
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_controller = controller;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize100() {
        TrajectoryVisualization.setViz(m_trajectory);
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(m_trajectory));
        m_controller.setTrajectory(iter);
        m_timer.reset();
        m_timer.start();
        m_goal = m_trajectory.getLastPoint().state().state().getPose();

    }

    @Override
    public void execute100(double dt) {
        final double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds currentRobotRelativeSpeed = m_robotDrive.speeds(dt);
        Twist2d robotRelativeVelocity = GeometryUtil.toTwist2d(currentRobotRelativeSpeed);
        ChassisSpeeds output = m_controller.update(now, currentPose, robotRelativeVelocity);

        t.log(Level.TRACE, m_name, "chassis speeds", output);
        t.log(Level.TRACE, m_name, "goal", m_goal);
        t.log(Level.TRACE, m_name, "THETA ERROR",
                Math.abs(m_goal.getRotation().getRadians() - m_robotDrive.getPose().getRotation().getRadians()));
        t.log(Level.TRACE, m_name, "FINSIHED", false);

        // if(m_controller.isDone() || m_timer.get() >
        // m_trajectory.getLastPoint().state().getTimeS()){

        // m_robotDrive.setChassisSpeedsNormally(new ChassisSpeeds(), dt);
        // return;
        // }

        m_robotDrive.setChassisSpeedsNormally(output, dt);
    }

    @Override
    public boolean isFinished() {

        // if( Math.abs(m_goal.getX() - m_robotDrive.getPose().getX()) < 0.1){
        // if( Math.abs(m_goal.getY() - m_robotDrive.getPose().getY()) < 0.1){
        // if(Math.abs(m_goal.getRotation().getRadians() -
        // m_robotDrive.getPose().getRotation().getRadians()) < 0.1){
        // if(m_controller.isDone()){
        // return m_timer.get() > m_trajectory.getLastPoint().state().getTimeS() +
        // m_timeBuffer;
        // // return true;
        // }
        // }
        // }
        // }

        if (m_controller.isDone()) {
            return true;
        }

        return false;

        // return m_timer.get() > m_trajectory.getLastPoint().state().getTimeS() +
        // m_timeBuffer;
        // return m_controller.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        t.log(Level.TRACE, m_name, "FINSIHED", true);
        m_robotDrive.stop();
        System.out.println("I HAVE FINISHED");
        m_timer.stop();
        TrajectoryVisualization.clear();
    }

    public Trajectory100 getTrajectory() {
        return m_trajectory;
    }
}

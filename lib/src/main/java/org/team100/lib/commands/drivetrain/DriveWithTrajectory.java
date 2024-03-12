package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
// import org.team100.lib.json.JSONParser;
// import org.team100.lib.json.TrajectoryList;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryVisualization;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class DriveWithTrajectory extends Command100 {
  /** Creates a new DriveWithTrajectory. */
  private final SwerveDriveSubsystem m_swerve;
  private final TrajectoryPlanner m_planner;
  private final DriveMotionController m_controller;
  private final SwerveKinodynamics m_limits;
  private final String m_fileName;
  private static final Telemetry t = Telemetry.get();

  public DriveWithTrajectory(SwerveDriveSubsystem drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits,
            String fileName) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_swerve = drivetrain;
        m_planner = planner;
        m_controller = controller;
        m_limits = limits;
        m_fileName = fileName;
        addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize100() {
    // System.out.println("DRIVE WITH TRAJEC STARTING");

    TrajectoryList trajectoryList = JSONParser.getTrajectoryList(m_fileName);
    // TrajectoryList trajectoryList = new TrajectoryList(null, null);

    trajectoryList.removeLastIndex();
    List<Pose2d> poses = getWaypoints(trajectoryList.getPoseArray());
    List<Rotation2d> headings = trajectoryList.getRotationArray();

    // headings.remove(0);
    // headings.add(new Rotation2d());

    // System.out.println("POSE AFTER LENGTH: " + poses.size());
    // System.out.println("HEADINGS LENGTH: " + headings.size());
    // System.out.println("POSE B4 LENGTH: " +
        // trajectoryList.getPoseArray().size());

    // System.out.println("THIS IS THE ORIGINAL TRAJEC LIST" +
        // trajectoryList.getPoseArray());

    List<TimingConstraint> constraints = new TimingConstraintFactory(m_limits).allGood();

    double max_vel = 5;
    double max_acc = 5;
    double start_vel = 0;
    double end_vel = 0;

    Trajectory100 trajectory = m_planner
                .generateTrajectory(
                        false,
                        poses,
                        headings,
                        constraints,
                        0,
                        0,
                        5,
                        5);

    TrajectoryVisualization.setViz(trajectory);

    TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));

    m_controller.setTrajectory(iter);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute100(double dt) {
     double now = Timer.getFPGATimestamp();
     Pose2d currentPose = m_swerve.getPose();
     ChassisSpeeds currentSpeed = m_swerve.speeds(dt);
     Twist2d velocity = new Twist2d(
                currentSpeed.vxMetersPerSecond,
                currentSpeed.vyMetersPerSecond,
                currentSpeed.omegaRadiansPerSecond);
     ChassisSpeeds output = m_controller.update(now, currentPose, velocity);
        
     t.log(Level.DEBUG, m_name, "chassis speeds", output);
     DriveUtil.checkSpeeds(output);
     m_swerve.setChassisSpeeds(output, dt);
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("DRIVE WITH TRAJEC ENDING");
    m_swerve.stop();
    TrajectoryVisualization.clear();
  }

  @Override
  public boolean isFinished() {
   return m_controller.isDone();
  }

  private static List<Pose2d> getWaypoints(List<Pose2d> m) {
        List<Pose2d> waypointsM = new ArrayList<>();
        // System.out.println("THIS IS M" + m);
        for (int i = 0; i < m.size() - 1; i += 1) {
            // Handles case for the last trajectory point
            // if(i == m.size() -1 ){
            // System.out.println("GOING");
            // Translation2d t0 = m.get(i).getTranslation();
            // Translation2d t1 = m.get(i-1).getTranslation();
            // Rotation2d theta = t1.minus(t0).getAngle();
            // System.out.println(new Pose2d(t0, theta));
            // waypointsM.add(new Pose2d(t0, theta));
            // } else {
                Translation2d t0 = m.get(i).getTranslation();
                Translation2d t1 = m.get(i + 1).getTranslation();
                Rotation2d theta = t1.minus(t0).getAngle();
                waypointsM.add(new Pose2d(t0, theta));
            // }
            
}

        // Last Value
        Translation2d t0 = m.get(m.size() - 1).getTranslation();
        Translation2d t1 = m.get(m.size() - 2).getTranslation();
        Rotation2d theta = t1.minus(t0).getAngle();

        waypointsM.add(new Pose2d(t0, theta));

        return waypointsM;
        
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
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


public class DriveWithWaypoints extends Command100 {
  /** Creates a new DriveWithTrajectory. */
  private final SwerveDriveSubsystem m_swerve;
  private final TrajectoryPlanner m_planner;
  private final DriveMotionController m_controller;
  private final SwerveKinodynamics m_limits;
  private static final Telemetry t = Telemetry.get();
//   private final List<Pose2d> m_waypoints;
//   private final List<Rotation2d> m_headings;
  private final Supplier<List<Pose2d>> m_goal;



  public DriveWithWaypoints(SwerveDriveSubsystem drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits,
            Supplier<List<Pose2d>> goal) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_swerve = drivetrain;
        m_planner = planner;
        m_controller = controller;
        m_limits = limits;
        m_goal = goal;
        // m_waypoints = waypoints;
        // m_headings = headings;

        

        addRequirements(m_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize100() {

    final Pose2d start = m_swerve.getPose();
    final Pose2d end = m_goal.get().get(0);
    

    List<Pose2d> newWaypointM = new ArrayList<>(m_goal.get());
    newWaypointM.add(0, start);

    List<Rotation2d> headings = new ArrayList<>();

    for(int i = 0; i < newWaypointM.size(); i++){
      headings.add(newWaypointM.get(i).getRotation());
    }

    newWaypointM = getWaypointsList(newWaypointM);

    // System.out.println("NEW WAYPOINT LENGHT" + newWaypointM.size());
    // System.out.println("HEADINGS LENGHT" + headings.size());


    // List<Pose2d> waypointsM = getWaypoints(start, end);
    

    List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(m_limits));

    double max_vel = 5;
    double max_acc = 5;
    double start_vel = 0;
    double end_vel = 0;

    // System.out.println(newWaypointM);

    Trajectory100 trajectory = m_planner
                .generateTrajectory(
                        false,
                        newWaypointM,
                        headings,
                        constraints,
                        0,
                        0,
                        5,
                        5);

    // TrajectosryVisualization.setViz(trajectory);

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
    TrajectoryVisualization.clear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return m_controller.isDone();
  }

  private static List<Pose2d> getWaypointsList(List<Pose2d> m) {
      // Translation2d t0 = m.get(0).getTranslation();
      // Translation2d t1 = m.get(1).getTranslation();
      // Rotation2d theta = t1.minus(t0).getAngle();
      // return List.of(
      //         new Pose2d(t0, theta),
      //         new Pose2d(t1, theta));
    
      List<Pose2d> waypointsM = new ArrayList<>();
      for(int i = 0; i < m.size()-1; i+=1){
          Translation2d t0 = m.get(i).getTranslation();
          Translation2d t1 = m.get(i+1).getTranslation();
          Rotation2d theta = t1.minus(t0).getAngle();
          waypointsM.add(new Pose2d(t0, theta));
      }

      Translation2d t0 = m.get(m.size()-1).getTranslation();
      Translation2d t1 = m.get(m.size()-2).getTranslation();
      Rotation2d theta = t0.minus(t1).getAngle();
      // double newTheta = 180 - theta.getDegrees();
      waypointsM.add(new Pose2d(t0, theta));  
            

      return waypointsM;
        
  }

    
   
}

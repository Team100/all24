// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands.drivetrain;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.team100.lib.JSON.JSONParser;
import org.team100.lib.JSON.TrajectoryList;
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

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.signals.Licensing_IsSeasonPassedValue;
import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithWaypoints extends Command100 {
  /** Creates a new DriveWithTrajectory. */
  private final SwerveDriveSubsystem m_swerve;
  private final TrajectoryPlanner m_planner;
  private final DriveMotionController m_controller;
  private final SwerveKinodynamics m_limits;
  private static final Telemetry t = Telemetry.get();
  private final List<Pose2d> m_waypoints;
  private final List<Rotation2d> m_headings;


  public DriveWithWaypoints(SwerveDriveSubsystem drivetrain,
            TrajectoryPlanner planner,
            DriveMotionController controller,
            SwerveKinodynamics limits,
            List<Pose2d> waypoints,
            List<Rotation2d> headings) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_swerve = drivetrain;
        m_planner = planner;
        m_controller = controller;
        m_limits = limits;
        m_waypoints = waypoints;
        m_headings = headings;

        

        addRequirements(m_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize100() {

    List<Pose2d> internalWaypoints = m_waypoints;
    List<Rotation2d> internalHeadings = m_headings;
    
    internalWaypoints.add(0, m_swerve.getPose());
    internalHeadings.add(0, m_swerve.getPose().getRotation());""?                                                                                                             m                                                                                                                                                                                                                                                                                                                    
    
    List<Pose2d> poses = new ArrayList<>();

    System.out.println("WAYPOINTS INTERNA:" + internalWaypoints);
    System.out.println("WAYPOINTS INTERNA SIZE:" + internalWaypoints.size());

    System.out.println("WAYPOINTS GLOBAL" + m_waypoints);
    System.out.println("WAYPOINTS INTERNA SIZE:" + m_waypoints.size());


    for(int i = 0; i < internalWaypoints.size(); i+=2){

        List<Pose2d> posi = getWaypoints(internalWaypoints.get(i), internalWaypoints.get(i + 1));
        poses.add(posi.get(0));
        poses.add(posi.get(1));
      
    }
      
    

    List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(m_limits));

    double max_vel = 5;
    double max_acc = 5;
    double start_vel = 0;
    double end_vel = 0;

    Trajectory100 trajectory = m_planner
                .generateTrajectory(
                        false,
                        internalWaypoints,
                        internalHeadings,
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

  private static List<Pose2d> getWaypoints(List<Pose2d> m) {
        List<Pose2d> waypointsM = new ArrayList<>();
        // System.out.println("THIS IS M" + m);
        for(int i = 0; i < m.size()-1; i+=1){
            //Handles case for the last trajectory point
            // if(i == m.size() -1 ){
            //     System.out.println("GOING");
            //     Translation2d t0 = m.get(i).getTranslation();
            //     Translation2d t1 = m.get(i-1).getTranslation();
            //     Rotation2d theta = t1.minus(t0).getAngle();
            //     System.out.println(new Pose2d(t0, theta));
            //     waypointsM.add(new Pose2d(t0, theta));
            // } else {
                Translation2d t0 = m.get(i).getTranslation();
                Translation2d t1 = m.get(i+1).getTranslation();
                Rotation2d theta = t1.minus(t0).getAngle();
                waypointsM.add(new Pose2d(t0, theta));
            // }
            

        }

        //Last Value
        Translation2d t0 = m.get(m.size()-1).getTranslation();
        Translation2d t1 = m.get(m.size()-2).getTranslation();
        Rotation2d theta = t1.minus(t0).getAngle();

        waypointsM.add(new Pose2d(t0, theta));

        return waypointsM;
        
    }

    private static List<Pose2d> getWaypoints(Pose2d p0, Pose2d p1) {
      Translation2d t0 = p0.getTranslation();
      Translation2d t1 = p1.getTranslation();
      Rotation2d theta = t1.minus(t0).getAngle();
      return List.of(
              new Pose2d(t0, theta),
              new Pose2d(t1, theta));
  }
   
}

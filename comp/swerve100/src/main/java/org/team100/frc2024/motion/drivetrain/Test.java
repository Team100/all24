// package org.team100.frc2024.motion.drivetrain;

// import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
// import org.team100.lib.sensors.HeadingInterface;

// import com.pathplanner.lib.commands.FollowPathHolonomic;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// import edu.wpi.first.wpilibj2.command.Command;

// public class Test {
    
//     public Command followPathCommand(SwerveDriveSubsystem m_drive, HeadingInterface m_heading){
//     PathPlannerPath path = PathPlannerPath.fromPathFile("");

//     // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
//     return new FollowPathWithEvents(
//         new FollowPathHolonomic(
//             path,
//             () -> m_drive.getPose(), // Robot pose supplier
//             () -> m_drive.speeds(0.02), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//             m_drive.m_swerveLocal::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//             new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//             4.5, // Max module speed, in m/s
//             0.4, // Drive base radius in meters. Distance from robot center to furthest module.
//             new ReplanningConfig(), // Default path replanning config. See the API for the options here
//             ()-> false,
//             m_drive), // Reference to this subsystem to set requirements
        
//         path, // FollowPathWithEvents also requires the path
//         () -> m_drive.getPose() // FollowPathWithEvents also requires the robot pose supplier
//     );
// }
// }

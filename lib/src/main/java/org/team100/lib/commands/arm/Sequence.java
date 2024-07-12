package org.team100.lib.commands.arm;

import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Sequence of arm trajectories.
 * 
 * These paint a square in cartesian space.
 */
public class Sequence extends SequentialCommandGroup {

    public Sequence(SupplierLogger parent, ArmSubsystem armSubsystem, ArmKinematics armKinematicsM) {

        addCommands(new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(.6, .6)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(1, .6)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(1, 1)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(.6, 1)),
                new ArmTrajectoryCommand(parent, armSubsystem, armKinematicsM, new Translation2d(.6, .6)));
    }
}

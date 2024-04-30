package org.team100.commands;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {
    private final RobotAssembly m_robot;

    public NonplayerDefault(RobotAssembly robot) {
        m_robot = robot;
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public void execute() {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        v = v.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        v = v.plus(Tactics.avoidEdges(m_robot.getPose()));
        v = v.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        v = v.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        v = v.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));
        m_robot.getDriveSubsystem().drive(v);
    }
}

package org.team100.commands;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

public class NonplayerDefault extends Command {

    private final RobotAssembly m_robot;
    private long timeMicros;

    
    public NonplayerDefault(RobotAssembly robot) {
        m_robot = robot;
        timeMicros = RobotController.getFPGATime();
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public void execute() {
        FieldRelativeAcceleration a = new FieldRelativeAcceleration(0, 0, 0);
        a = a.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        a = a.plus(Tactics.avoidEdges(m_robot.getPose()));
        a = a.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        a = a.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        a = a.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));

                long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;

        FieldRelativeVelocity v = m_robot.getVelocity();
        FieldRelativeVelocity dv = a.integrate(dtSec);
        v = v.plus(dv);
        m_robot.getDriveSubsystem().drive(v);


    }
}

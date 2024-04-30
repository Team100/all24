package org.team100.commands;

import org.team100.alliance.Alliance;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj2.command.Command;

/** Drive towards the nearest note and take it. */
public class Steal extends Command {
    private final Alliance m_alliance;
    private final RobotAssembly m_robot;

    public Steal(Alliance alliance, RobotAssembly robot) {
        m_alliance = alliance;
        m_robot = robot;
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Steal: " + m_robot.getName();
    }

    @Override
    public void execute() {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        v = v.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        v = v.plus(Tactics.avoidEdges(m_robot.getPose()));
        v = v.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        v = v.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        v = v.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));
        // a = a.plus(goToGoal());
        m_robot.getDriveSubsystem().drive(v);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    // private void goToGoal() {
    //     //
    // }

}

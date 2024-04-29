package org.team100.commands;

import org.team100.alliance.Alliance;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

/** Drive towards the nearest note and take it. */
public class Steal extends Command {
    private final Alliance m_alliance;
    private final RobotAssembly m_robot;
    private long timeMicros;

    public Steal(Alliance alliance, RobotAssembly robot) {
        m_alliance = alliance;
        m_robot = robot;
        timeMicros = RobotController.getFPGATime();

        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Steal: " + m_robot.getName();
    }

    @Override
    public void execute() {

        FieldRelativeAcceleration a = new FieldRelativeAcceleration(0, 0, 0);

        a = a.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        a = a.plus(Tactics.avoidEdges(m_robot.getPose()));
        a = a.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        a = a.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        a = a.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));
        // a = a.plus(goToGoal());

        long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;

        FieldRelativeVelocity v = m_robot.getVelocity();
        FieldRelativeVelocity dv = a.integrate(dtSec);
        v = v.plus(dv);
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

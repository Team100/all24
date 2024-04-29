package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.alliance.Alliance;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreAmp extends Command {
    private static final int kAmpAttraction = 50;
    private final Alliance m_alliance;
    private final RobotAssembly m_robot;

    public ScoreAmp(Alliance alliance, RobotAssembly robot) {
        m_alliance = alliance;
        m_robot = robot;
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Score Amp: " + m_robot.getName();
    }

    @Override
    public void execute() {
        Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity());
        Tactics.avoidEdges(m_robot.getPose());
        Tactics.avoidSubwoofers(m_robot.getPose());
        Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings());
        Tactics.robotRepulsion(m_robot.getPose(),  m_robot.recentSightings());
        goToGoal();
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.ampPosition();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, goal);
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        return translationError < 0.1 && Math.abs(rotationError) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    private void goToGoal() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.ampPosition();
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        positionError = new Vector2(
                Math.min(1, positionError.x),
                Math.min(1, positionError.y));
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        m_robot.getRobotBody().applyForce(positionError.product(kAmpAttraction));
        m_robot.getRobotBody().applyTorque(rotationError * 10);
    }
}

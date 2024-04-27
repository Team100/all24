package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.robot.RobotSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreSpeaker extends Command {
    private static final int kSpeakerAttraction = 50;
    private final Alliance m_alliance;
    private final RobotSubsystem m_robot;
    private final Tactics m_tactics;

    public ScoreSpeaker(Alliance alliance, RobotSubsystem robot) {
        m_alliance = alliance;
        m_robot = robot;
        m_tactics = new Tactics(robot);
        addRequirements(robot);
    }

    @Override
    public String getName() {
        return "Score Speaker: " + m_robot.getName();
    }

    /** TODO: replace with a more general driving plan */
    @Override
    public void execute() {
        m_tactics.avoidObstacles();
        m_tactics.avoidEdges();
        m_tactics.avoidSubwoofers();
        m_tactics.steerAroundRobots();
        m_tactics.robotRepulsion();
        goToGoal();
    }

    /**
     * speaker position tolerance is loose but angle is not
     */
    @Override
    public boolean isFinished() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.shootingPosition();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, goal);
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        return translationError < 0.5 && Math.abs(rotationError) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    private void goToGoal() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.shootingPosition();
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        positionError = new Vector2(
                Math.min(1, positionError.x),
                Math.min(1, positionError.y));
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        m_robot.getRobotBody().applyForce(positionError.product(kSpeakerAttraction));
        m_robot.getRobotBody().applyTorque(rotationError * 10);
    }

}

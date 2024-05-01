package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.alliance.Alliance;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives to a good spot for shooting.
 * 
 * TODO: extra a "drive to X" command
 */
public class DriveToSpeaker extends Command {
    private static final int kAngularP = 10;
    private static final int kCartesianP = 50;
    /** m_alliance may be null. */
    private final Alliance m_alliance;
    private final RobotAssembly m_robot;

    /** alliance may be null. */
    public DriveToSpeaker(Alliance alliance, RobotAssembly robot) {
        m_alliance = alliance;
        m_robot = robot;
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Score Speaker: " + m_robot.getName();
    }

    /** TODO: replace with a more general driving plan */
    @Override
    public void execute() {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        v = v.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        v = v.plus(Tactics.avoidEdges(m_robot.getPose()));
        v = v.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        v = v.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        v = v.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));
        v = v.plus(goToGoal());
        m_robot.getDriveSubsystem().drive(v);
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
        double velocity = m_robot.getVelocity().norm();

        return translationError < 0.5
                && Math.abs(rotationError) < 0.05
                && velocity < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_alliance != null)
            m_alliance.onEnd(m_robot, this);
    }

    /** Proportional feedback with a limiter. */
    private FieldRelativeVelocity goToGoal() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.shootingPosition();
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        final int maxError = 1;
        positionError = new Vector2(
                MathUtil.clamp(positionError.x, -maxError, maxError),
                MathUtil.clamp(positionError.y, -maxError, maxError));
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB);
    }

}

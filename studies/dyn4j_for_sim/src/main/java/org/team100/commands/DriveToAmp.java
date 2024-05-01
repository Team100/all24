package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** TODO: extract a "drive to X" command */
public class DriveToAmp extends Command {
    private static final int kAngularP = 10;
    // try a bit more P
    private static final int kCartesianP = 75;
    private final RobotAssembly m_robot;

    public DriveToAmp(RobotAssembly robot) {
        m_robot = robot;
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Score Amp: " + m_robot.getName();
    }

    @Override
    public void execute() {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        v = v.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        // turn off edge repulsion for now
        // v = v.plus(Tactics.avoidEdges(m_robot.getPose()));
        v = v.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        v = v.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        v = v.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));
        v = v.plus(goToGoal());
        m_robot.getDriveSubsystem().drive(v);
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

    /** Proportional feedback with a limiter. */
    private FieldRelativeVelocity goToGoal() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.ampPosition();
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

package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.alliance.Alliance;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

/** Drives to a good spot for shooting. */
public class DriveToSpeaker extends Command {
    private static final int kSpeakerAttraction = 50;
    private final Alliance m_alliance;
    private final RobotAssembly m_robot;
    private long timeMicros;


    public DriveToSpeaker(Alliance alliance, RobotAssembly robot) {
        m_alliance = alliance;
        m_robot = robot;
        timeMicros = RobotController.getFPGATime();

        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Score Speaker: " + m_robot.getName();
    }

    /** TODO: replace with a more general driving plan */
    @Override
    public void execute() {
        FieldRelativeAcceleration a = new FieldRelativeAcceleration(0, 0, 0);
        a = a.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        a = a.plus(Tactics.avoidEdges(m_robot.getPose()));
        a = a.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        a = a.plus(Tactics.steerAroundRobots(m_robot.getPose(), m_robot.getVelocity(), m_robot.recentSightings()));
        a = a.plus(Tactics.robotRepulsion(m_robot.getPose(), m_robot.recentSightings()));
        a = a.plus(goToGoal());


                long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;

        FieldRelativeVelocity v = m_robot.getVelocity();
        FieldRelativeVelocity dv = a.integrate(dtSec);
        v = v.plus(dv);
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
        return translationError < 0.5 && Math.abs(rotationError) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    private FieldRelativeAcceleration goToGoal() {
        Pose2d pose = m_robot.getPose();
        Pose2d goal = m_robot.shootingPosition();
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        positionError = new Vector2(
                Math.min(1, positionError.x),
                Math.min(1, positionError.y));
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        
        Vector2 accel = positionError.product(kSpeakerAttraction);
        // m_robot.getRobotBody().applyForce(accel);
        // m_robot.getRobotBody().applyTorque(rotationError * 10);
        return new FieldRelativeAcceleration(accel.x, accel.y, rotationError*10);
    }

}

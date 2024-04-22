package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.robot.RobotSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreSpeaker extends Command {
    private final RobotSubsystem m_robot;
    private final Tactics m_tactics;

    public ScoreSpeaker(RobotSubsystem robot) {
        m_robot = robot;
        m_tactics = new Tactics(robot);
        addRequirements(robot);
    }

    /** TODO: replace with a more general driving plan */
    protected void driveToSpeaker() {
        Vector2 position = m_robot.getRobotBody().transform.getTranslation();
        Vector2 positionError = position.to(m_robot.shootingPosition());
        double angle = m_robot.getRobotBody().transform.getRotationAngle();
        // TODO: remove this magic angle
        double angleError = MathUtil.angleModulus(m_robot.getRobotBody().shootingAngle() - angle);
        // speaker position tolerance is loose but angle is not
        if (Math.abs(positionError.x) < 0.5
                && Math.abs(positionError.y) < 0.5
                && Math.abs(angleError) < 0.05) {
            // successful score
            m_robot.getRobotBody().nextGoal();
        } else {
            // keep trying
            m_robot.getRobotBody().applyForce(positionError.setMagnitude(200));
            m_robot.getRobotBody().applyTorque(angleError * 10);
        }
    }



}

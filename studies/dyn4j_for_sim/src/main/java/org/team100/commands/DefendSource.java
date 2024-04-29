package org.team100.commands;

import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.alliance.Alliance;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.RobotAssembly;
import org.team100.subsystems.CameraSubsystem.RobotSighting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stay between opponents and their source.
 * 
 * Never finishes.
 */
public class DefendSource extends Command {
    private static final int kDefensePushing = 50;
    private static final int kCornerRepulsion = -10;
    private static final int kWaitingAttraction = 1000;
    private final Alliance m_alliance;
    private final RobotAssembly m_robot;
    private long timeMicros;

    public DefendSource(Alliance alliance, RobotAssembly robot) {
        m_alliance = alliance;
        m_robot = robot;
        timeMicros = RobotController.getFPGATime();
        addRequirements(robot.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Defend Source: " + m_robot.getName();
    }

    @Override
    public void execute() {
        FieldRelativeAcceleration a = new FieldRelativeAcceleration(0, 0, 0);
        a = a.plus(Tactics.avoidObstacles(m_robot.getPose(), m_robot.getVelocity()));
        a = a.plus(Tactics.avoidEdges(m_robot.getPose()));
        a = a.plus(Tactics.avoidSubwoofers(m_robot.getPose()));
        // it's ok to collide with robots
        a = a.plus(work(
                m_robot.getPose(),
                m_robot.getDefenderPosition(),
                m_robot.getOpponentSourcePosition(),
                m_robot.recentSightings()));

        long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;

        FieldRelativeVelocity v = m_robot.getVelocity();
        FieldRelativeVelocity dv = a.integrate(dtSec);
        v = v.plus(dv);
        m_robot.getDriveSubsystem().drive(v);
    }

    /** Never finishes but might be interrupted. */
    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    /**
     * If no robots are around, wait near the source. If there is a foe, stay
     * between it and the source. Avoid getting too close to the source.
     */
    private static FieldRelativeAcceleration work(
            Pose2d pose,
            Pose2d defenderPosition,
            Pose2d opponentSourcePosition,
            NavigableMap<Double, RobotSighting> recentSightings) {
        FieldRelativeAcceleration a = new FieldRelativeAcceleration(0, 0, 0);

        Vector2 position = new Vector2(pose.getX(), pose.getY());

        // attract to the waiting spot
        FieldRelativeDelta toWaitingSpot = FieldRelativeDelta.delta(pose, defenderPosition)
                .limit(1, 1);
        a = a.plus(new FieldRelativeAcceleration(
                toWaitingSpot.getX(),
                toWaitingSpot.getY(),
                0).times(kWaitingAttraction));

        // repel from the corner, and don't chase opponents, if too close
        FieldRelativeDelta toCorner = FieldRelativeDelta.delta(pose, opponentSourcePosition);
        if (toCorner.getTranslation().getNorm() < 2.5) {
            a = a.plus(new FieldRelativeAcceleration(
                    toCorner.getX(),
                    toCorner.getY(),
                    0).times(
                            kCornerRepulsion
                                    / (toCorner.getTranslation().getNorm() * toCorner.getTranslation().getNorm())));
            return a;
        }
        // give up if too far
        if (toCorner.getTranslation().getNorm() > 4) {
            return a;
        }
        // TODO: if there's an opponent nearby, stay between it and the corner.
        // NavigableMap<Double, RobotSighting> recentSightings =
        // m_robot.recentSightings();
        for (Entry<Double, RobotSighting> mostRecent : recentSightings.entrySet()) {
            RobotSighting mostRecentSighting = mostRecent.getValue();
            // don't try to defend friends
            if (mostRecentSighting.friend())
                continue;
            Translation2d mostRecentPosition = mostRecentSighting.position();

            double distance = pose.getTranslation().getDistance(mostRecentPosition);
            if (distance > 4) {
                // don't react to far-away obstacles
                continue;
            }
            double fieldDistance = defenderPosition.getTranslation().getDistance(mostRecentPosition);
            if (fieldDistance > 3) {
                // don't chase it too far
                continue;
            }
            // for now just drive hard towards it
            // TODO: figure out which side to push on
            Vector2 toOpponent = position.to(
                    new Vector2(mostRecentPosition.getX(), mostRecentPosition.getY()));
            Vector2 force = toOpponent.product(
                    kDefensePushing / toOpponent.getMagnitudeSquared());
            a = a.plus(new FieldRelativeAcceleration(force.x, force.y, 0));
            break;
        }
        return a;
    }

}

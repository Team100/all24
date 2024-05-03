package org.team100.commands;

import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.RobotSighting;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stay between opponents and their source.
 * 
 * Never finishes.
 */
public class DefendSource extends Command {
    // TODO: get these from kinodynamics
    private static final double kMaxVelocity = 5; // m/s
    private static final double kMaxOmega = 10; // rad/s

    private static final int kDefensePushing = 5;
    private static final int kCornerRepulsion = -5;
    private static final int kWaitingAttraction = 5;
    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final Tactics m_tactics;

    public DefendSource(DriveSubsystem drive, CameraSubsystem camera) {
        m_drive = drive;
        m_camera = camera;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        FieldRelativeVelocity desired = work(
                m_drive.getPose(),
                m_drive.getRobotBody().defenderPosition(),
                m_drive.getRobotBody().opponentSourcePosition(),
                m_camera.recentSightings());
        ForceViz.put("desired", m_drive.getPose(), desired);
        FieldRelativeVelocity v = m_tactics.apply(desired, true, false, false);
        v = v.plus(desired);
        v = v.clamp(kMaxVelocity, kMaxOmega);
        m_drive.drive(v);
    }

    /**
     * If no robots are around, wait near the source. If there is a foe, stay
     * between it and the source. Avoid getting too close to the source.
     */
    private static FieldRelativeVelocity work(
            Pose2d pose,
            Pose2d defenderPosition,
            Pose2d opponentSourcePosition,
            NavigableMap<Double, RobotSighting> recentSightings) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);

        Vector2 position = new Vector2(pose.getX(), pose.getY());

        // attract to the waiting spot
        FieldRelativeDelta toWaitingSpot = FieldRelativeDelta.delta(pose, defenderPosition)
                .limit(1, 1);
        v = v.plus(new FieldRelativeVelocity(
                toWaitingSpot.getX(),
                toWaitingSpot.getY(),
                0).times(kWaitingAttraction));

        // repel from the corner, and don't chase opponents, if too close
        FieldRelativeDelta toCorner = FieldRelativeDelta.delta(pose, opponentSourcePosition);
        if (toCorner.getTranslation().getNorm() < 2.5) {
            double magnitude = kCornerRepulsion
                    / (toCorner.getTranslation().getNorm() * toCorner.getTranslation().getNorm());
            v = v.plus(new FieldRelativeVelocity(
                    toCorner.getX(),
                    toCorner.getY(),
                    0).times(magnitude));
            return v;
        }
        // give up if too far
        if (toCorner.getTranslation().getNorm() > 4) {
            return v;
        }
        // TODO: if there's an opponent nearby, stay between it and the corner.
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
            v = v.plus(new FieldRelativeVelocity(force.x, force.y, 0));
            break;
        }
        return v;
    }

}

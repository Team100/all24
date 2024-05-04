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

    /** push towards the opponent */
    private static final double kDefensePushing = 5;
    /** absolutely do not get pushed into the protected area */
    private static final double kCornerRepulsion = -50;
    /** get back to the spot fast if nothing else is happening */
    private static final double kWaitingAttraction = 5;
    /** try very hard stay between the opponent and the corner */
    private static final double kCorner = 50;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DefendSource(DriveSubsystem drive, CameraSubsystem camera, boolean debug) {
        m_drive = drive;
        m_camera = camera;
        m_debug = debug;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("Defend");
        Pose2d pose = m_drive.getPose();
        if (m_debug)
            System.out.printf(" pose (%5.2f,%5.2f)", pose.getX(), pose.getY());
        FieldRelativeVelocity desired = work(
                pose,
                m_drive.getRobotBody().defenderPosition(),
                m_drive.getRobotBody().opponentSourcePosition(),
                m_camera.recentSightings(),
                m_debug);
        if (m_debug)
            ForceViz.put("desired", pose, desired);
        if (m_debug)
            System.out.printf(" desired %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired, false, true, false, m_debug);
        if (m_debug)
            System.out.printf(" tactics %s", v);
        v = v.plus(desired);
        v = v.clamp(kMaxVelocity, kMaxOmega);
        if (m_debug)
            System.out.printf(" final %s\n", v);
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
            NavigableMap<Double, RobotSighting> recentSightings,
            boolean debug) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);

        Vector2 myPosition = new Vector2(pose.getX(), pose.getY());

        // attract to the waiting spot but only if nothing else is happening
        FieldRelativeDelta toWaitingSpot = FieldRelativeDelta.delta(pose, defenderPosition)
                .limit(1, 1);
        FieldRelativeVelocity waiting = new FieldRelativeVelocity(
                toWaitingSpot.getX(),
                toWaitingSpot.getY(),
                0).times(kWaitingAttraction);
        if (debug)
            System.out.printf(" waiting %s", waiting);
        v = v.plus(waiting);

        // repel from the corner (1/r), and don't chase opponents, if too close
        // (to avoid fouling)
        FieldRelativeDelta toCorner = FieldRelativeDelta.delta(pose, opponentSourcePosition);
        if (toCorner.getTranslation().getNorm() < 1.5) {
            double magnitude = kCornerRepulsion
                    / (toCorner.getTranslation().getNorm() * toCorner.getTranslation().getNorm());
            FieldRelativeVelocity repel = new FieldRelativeVelocity(
                    toCorner.getX(),
                    toCorner.getY(),
                    0).times(magnitude);
            if (debug)
                System.out.printf(" repel %s", repel);
            // v = v.plus(repel);
            v = repel;
            return v;
        }
        // give up if too far
        if (toCorner.getTranslation().getNorm() > 6) {
            System.out.print(" too far");
            return v;
        }
        //
        // TODO: if there's an opponent nearby, stay between it and the corner.
        for (Entry<Double, RobotSighting> mostRecent : recentSightings.entrySet()) {
            RobotSighting mostRecentSighting = mostRecent.getValue();
            // don't try to defend friends
            // TODO: stay out of their way
            if (mostRecentSighting.friend())
                continue;
            Translation2d foe = mostRecentSighting.position();

            double distanceFromMe = pose.getTranslation().getDistance(foe);
            if (distanceFromMe > 6) {
                // don't react to far-away obstacles
                continue;
            }
            double distanceFromSource = opponentSourcePosition.getTranslation().getDistance(foe);
            if (distanceFromSource > 8) {
                // don't chase it too far
                continue;
            }
            if (debug)
                System.out.printf(" foe (%5.2f, %5.2f)", foe.getX(), foe.getY());
            // drive towards the opponent
            Vector2 toOpponent = myPosition.to(
                    new Vector2(foe.getX(), foe.getY()));
            Vector2 force = toOpponent.product(
                    kDefensePushing / toOpponent.getMagnitudeSquared());
            FieldRelativeVelocity push = new FieldRelativeVelocity(force.x, force.y, 0);
            if (debug)
                System.out.printf(" push %s", push);
            // v = v.plus(push);
            v = push;
            // try to get to the line between the opponent and the source
            Translation2d sourceToFoe = foe.minus(opponentSourcePosition.getTranslation());
            sourceToFoe = sourceToFoe.div(sourceToFoe.getNorm());
            Translation2d sourceToMe = pose.getTranslation().minus(opponentSourcePosition.getTranslation());
            sourceToMe = sourceToMe.div(sourceToMe.getNorm());
            Translation2d toFoe = sourceToFoe.minus(sourceToMe);
            FieldRelativeVelocity corner = new FieldRelativeVelocity(toFoe.getX() * kCorner, toFoe.getY() * kCorner, 0);
            if (debug)
                System.out.printf(" corner %s", corner);
            v = v.plus(corner);

            break;
        }
        return v;
    }

}

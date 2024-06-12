package org.team100.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Foes try to pick from the source and score in the amp corner.
 * 
 * Foes should have human-readable names.
 * 
 * TODO: spin away if close to an opponent
 */
public class Foe extends RobotBody {
    /**
     * Intake is on the back, robot usually comes from ahead, so source pick
     * rotation is 0, i.e. arrive in reverse.
     */
    static final Pose2d kSource = new Pose2d(1, 2, new Rotation2d());
    /** This is the robot center when facing the amp */
    private static final Pose2d kAmpSpot = new Pose2d(14.7, 7.5, new Rotation2d(-Math.PI / 2));
    static final Pose2d kPassingSpot = new Pose2d(6.5, 1, new Rotation2d(0.65));
    static final Pose2d kDefendSpot = new Pose2d(13, 2, new Rotation2d(Math.PI));
    /** Center of the speaker: the target to shoot at. */
    private static final Translation2d kSpeaker = new Translation2d(16.541, 5.548);

    private final double m_yBias;

    /** Note: adds this to the world. */
    public Foe(String id, SimWorld world, double yBias, boolean debug) {
        super(id, world, debug);
        m_yBias = yBias;
    }

    @Override
    public boolean friend(RobotBody other) {
        // only foes are friends
        return other instanceof Foe;
    }

    @Override
    public Pose2d ampPosition() {
        return kAmpSpot;
    }

    @Override
    public Pose2d sourcePosition() {
        return kSource;
    }

    @Override
    public Pose2d opponentSourcePosition() {
        return Friend.kSource;
    }

    @Override
    public Pose2d defenderPosition() {
        return kDefendSpot;
    }

    @Override
    public Pose2d passingPosition() {
        return kPassingSpot;
    }

    @Override
    public Translation2d speakerPosition() {
        return kSpeaker;
    }

    @Override
    public double yBias() {
        return m_yBias;
    }

}

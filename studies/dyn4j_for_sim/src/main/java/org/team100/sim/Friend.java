package org.team100.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Friends try to pick from the source and score in the amp corner.
 * 
 * Friends should have human-readable names.
 * 
 * TODO: coordinate "lanes" among alliance-mates
 * TODO: spin away if close to an opponent
 */
public class Friend extends RobotBody {
    /**
     * Intake is on the back, robot usually comes from behind, so source pick
     * rotation target is 180, i.e. arrive in reverse.
     */
    static final Pose2d kSource = new Pose2d(15.5, 2, new Rotation2d(Math.PI));
    /** This is the robot center when facing the amp */
    static final Pose2d kAmpSpot = new Pose2d(1.840, 7.5, new Rotation2d(-Math.PI / 2));
    /** Shoot from about 3 meters away */
    static final Pose2d kShootingSpot = new Pose2d(3.0, 5.5, new Rotation2d(Math.PI));
    static final Pose2d kPassingSpot = new Pose2d(9.5, 1, new Rotation2d(3 * Math.PI / 4));
    static final Pose2d kDefendSpot = new Pose2d(3.5, 2, new Rotation2d());

    /** Note: adds this to the world. */
    public Friend(String id, SimWorld world, boolean debug) {
        super(id, world, debug);
    }

    @Override
    public boolean friend(RobotBody other) {
        // either friends or player
        return other instanceof Friend
                || other instanceof Player;
    }

    @Override
    public Pose2d ampPosition() {
        return kAmpSpot;
    }

    @Override
    public Pose2d shootingPosition() {
        return kShootingSpot;
    }

    @Override
    public Pose2d sourcePosition() {
        return kSource;
    }

    @Override
    public Pose2d opponentSourcePosition() {
        return Foe.kSource;
    }

    @Override
    public Pose2d defenderPosition() {
        return kDefendSpot;
    }

    @Override
    public Pose2d passingPosition() {
        return kPassingSpot;
    }
}

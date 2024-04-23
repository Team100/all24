package org.team100.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Foes try to pick from the source and score in the amp corner.
 * 
 * Foes should have human-readable names.
 * 
 * TODO: add defensive behavior
 * TODO: add two kinds of scoring
 * TODO: coordinate "lanes" among alliance-mates
 * TODO: spin away if close to an opponent
 */
public class Foe extends RobotBody {
    /** Spot for source pick */
    static final Pose2d kSource = new Pose2d(1, 2, new Rotation2d(Math.PI));
    /** This is the robot center when facing the amp */
    private static final Pose2d kAmpSpot = new Pose2d(14.7, 7.5, new Rotation2d(Math.PI / 2));
    /** Shoot from about 3 meters away */
    private static final Pose2d kShootingSpot = new Pose2d(13.5, 5.5, new Rotation2d());
    static final Pose2d kPassingSpot = new Pose2d(6.5, 1, new Rotation2d(Math.PI / 4));
    static final Pose2d kDefendSpot = new Pose2d(13, 2, new Rotation2d(Math.PI));

    public Foe(String id, SimWorld world) {
        super(id, world);
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
    public Pose2d shootingPosition() {
        return kShootingSpot;
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

}

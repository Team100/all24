package org.team100.sim;

import org.dyn4j.geometry.Vector2;

/**
 * Friends try to pick from the source and score in the amp corner.
 * 
 * Friends should have human-readable names.
 * 
 * TODO: add two kinds of scoring
 * TODO: coordinate "lanes" among alliance-mates
 * TODO: spin away if close to an opponent
 */
public class Friend extends RobotBody {
    static final Vector2 kSource = new Vector2(16, 0);
    /** This is the robot center when facing the amp */
    static final Vector2 kAmpSpot = new Vector2(1.840, 8.204)
            .sum(0, -kRobotSize / 2);
    /** Shoot from about 3 meters away */
    static final Vector2 kShootingSpot = new Vector2(3.0, 5.5);

    static final double kShootingAngle = Math.PI;

    public Friend(String id, SimWorld world) {
        super(id, world);
    }

    @Override
    public boolean friend(RobotBody other) {
        // either friends or player
        return other instanceof Friend
                || other instanceof Player;
    }

    @Override
    public Vector2 ampPosition() {
        return kAmpSpot;
    }

    @Override
    public Vector2 shootingPosition() {
        return kShootingSpot;
    }

    @Override
    public double shootingAngle() {
        return kShootingAngle;
    }

    @Override
    public Vector2 sourcePosition() {
        return kSource;
    }
}

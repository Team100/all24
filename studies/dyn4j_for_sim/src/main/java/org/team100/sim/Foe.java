package org.team100.sim;

import org.dyn4j.geometry.Vector2;

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
    private static final Vector2 kSource = new Vector2(0, 0);
    /** This is the robot center when facing the amp */
    private static final Vector2 kAmpSpot = new Vector2(14.698, 8.204)
            .sum(0, -kRobotSize / 2);
    /** Shoot from about 3 meters away */
    private static final Vector2 kShootingSpot = new Vector2(13.5, 5.5);
    private static final double kShootingAngle = 0;

    public Foe(String id, SimWorld world) {
        super(id, world);
    }

    @Override
    public boolean friend(RobotBody other) {
        // only foes are friends
        return other instanceof Foe;
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

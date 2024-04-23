package org.team100.sim;

import org.dyn4j.geometry.Vector2;

/**
 * Controls apply force and torque.
 */
public class Player extends RobotBody {

    public Player(SimWorld world, Goal initialGoal) {
        super("player", world, initialGoal);
    }

    @Override
    public void act() {
        // TODO: remove this
    }

    @Override
    public boolean friend(RobotBody other) {
        // only one player so only friends are friends
        return other instanceof Friend;
    }

    @Override
    public Vector2 ampPosition() {
        return Friend.kAmpSpot;
    }

    @Override
    public Vector2 shootingPosition() {
        return Friend.kShootingSpot;
    }

    @Override
    public double shootingAngle() {
        return Friend.kShootingAngle;
    }

    @Override
    public Vector2 sourcePosition() {
        return Friend.kSource;
    }
}

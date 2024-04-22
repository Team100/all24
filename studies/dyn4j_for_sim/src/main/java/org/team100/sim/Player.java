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

    }

    @Override
    public boolean friend(RobotBody other) {
        // only one player so only friends are friends
        return other instanceof Friend;
    }

    @Override
    Vector2 ampPosition() {
        return Friend.kAmpSpot;
    }

    @Override
    Vector2 shootingPosition() {
        return Friend.kShootingSpot;
    }

    double shootingAngle() {
        return Friend.kShootingAngle;
    }
}

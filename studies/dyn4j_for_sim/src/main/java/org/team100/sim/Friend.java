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
    private static final int kForce = 200;
    private static final int kTolerance = 2;
    private static final Vector2 kSource = new Vector2(16, 0);
    /** This is the robot center when facing the amp */
    static final Vector2 kAmpSpot = new Vector2(1.840, 8.204)
            .sum(0, -kRobotSize / 2);
    /** Shoot from about 3 meters away */
    static final Vector2 kShootingSpot = new Vector2(3.0, 5.5);

    static final double kShootingAngle = Math.PI;

    public Friend(String id, SimWorld world, Goal initialGoal) {
        super(id, world, initialGoal);
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
    public void act() {
        Vector2 position = getWorldCenter();
        switch (m_goal) {
            case PICK:
                Vector2 toPick = position.to(kSource);
                if (toPick.getMagnitude() < kTolerance) {
                    // successful pick, now go score
                    nextGoal();
                } else {
                    // keep trying
                    applyForce(toPick.setMagnitude(kForce));
                }
                break;
            case SCORE_AMP:
                driveToAmp();
                break;
            case SCORE_SPEAKER:
                driveToSpeaker();
                break;
            default:
                // do nothing
                break;
        }

        // look for nearby notes, brute force
        for (Body100 body : m_world.getBodies()) {
            if (body instanceof Note) {
                double distance = position.distance(body.getWorldCenter());
                if (distance > 0.3)
                    continue;
                // System.out.printf("%s %5.3f\n",
                // body.getClass().getSimpleName(), distance);
                // TODO: pick up?
            }
        }

        // avoidRobots();

    }
}

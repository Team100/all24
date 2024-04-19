package org.team100.sim;

import org.dyn4j.geometry.Vector2;

/**
 * Foes try to pick from the source and score in the amp corner.
 * 
 * TODO: add two kinds of scoring
 */
public class Friend extends RobotBody {
    private static final int kForce = 200;
    private static final int kTolerance = 2;
    private static final Vector2 kSource = new Vector2(16, 0);
    private static final Vector2 kAmpCorner = new Vector2(0, 8);

    private enum Goal {
        PICK, SCORE
    }

    private Goal m_goal;

    public Friend() {
        // initial goal
        m_goal = Goal.PICK;
    }

    @Override
    public void act() {
        Vector2 position = getWorldCenter();
        switch (m_goal) {
            case PICK:
                Vector2 toPick = position.to(kSource);
                if (toPick.getMagnitude() < kTolerance) {
                    // successful pick, now go score
                    m_goal = Goal.SCORE;
                } else {
                    // keep trying
                    applyForce(toPick.setMagnitude(kForce));
                }
                break;
            case SCORE:
                Vector2 toScore = position.to(kAmpCorner);
                if (toScore.getMagnitude() < kTolerance) {
                    // successful score, now go pick
                    m_goal = Goal.PICK;
                } else {
                    // keep trying
                    applyForce(toScore.setMagnitude(kForce));
                }
                break;
        }
    }
}

package frc.robot;

import org.dyn4j.geometry.Vector2;

public class Foe extends Nonplayer {

    public Foe() {
        // initial goal
        m_goal = Goal.PICK;
    }

    @Override
    public void act() {
        Vector2 p = getWorldCenter();
        switch (m_goal) {
            case PICK:
                // foes try to pick from the origin
                if (p.x < 2 && p.y < 2) {
                    // successful pick, now go score
                    m_goal = Goal.SCORE;
                } else {
                    // keep trying
                    applyForce(new Vector2(-200, -200));
                }
                break;
            case SCORE:
                // for now, "score" means the opposite corner
                if (p.x > 13 && p.y > 6) {
                    // successful score, now go pick
                    m_goal = Goal.PICK;
                } else {
                    // keep trying
                    applyForce(new Vector2(200, 200));
                }
                break;
        }
    }
}

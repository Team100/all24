package frc.robot;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

public class Friend extends Body100 {
    private static final int arealDensityKg_m2 = 100;
    private static final double robotSize = 0.75;

    enum Goal {
        PICK, SCORE
    }

    private Goal m_goal;

    public Friend() {
        BodyFixture fixture = addFixture(
                Geometry.createSquare(robotSize),
                arealDensityKg_m2,
                0.5,
                0.5);
        fixture.setRestitutionVelocity(0.0);
        setMass(MassType.NORMAL);
        setBullet(true);
        // initial goal
        m_goal = Goal.PICK;
    }

    @Override
    public void act() {
        Vector2 p = getWorldCenter();
        switch (m_goal) {
            case PICK:
                // friends try to pick from the far corner
                if (p.x > 14 && p.y < 2) {
                    // successful pick, now go score
                    m_goal = Goal.SCORE;
                } else {
                    // keep trying
                    applyForce(new Vector2(200, -200));
                }
                break;
            case SCORE:
                // for now, "score" means the opposite corner
                if (p.x < 3 && p.y > 6) {
                    // successful score, now go pick
                    m_goal = Goal.PICK;
                } else {
                    // keep trying
                    applyForce(new Vector2(-200, 200));
                }
                break;
        }
    }
}

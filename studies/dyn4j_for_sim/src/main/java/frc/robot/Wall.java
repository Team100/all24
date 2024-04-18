package frc.robot;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/** Walls cannot be correctly rendered in the simgui. */
public class Wall extends Body100 {
    public Wall(Convex convex) {
        // 0.5 friction is a guess
        // 0.5 restitution is a guess
        BodyFixture fixture = addFixture(convex, 1.0, 0.5, 0.5);
        // this means the springiness doesn't change with velocity
        fixture.setRestitutionVelocity(0.0);
        setMass(MassType.INFINITE);
    }

    @Override
    public void act() {
        // do nothing
    }
}

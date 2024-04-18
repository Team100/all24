package frc.robot;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

public class Obstacle extends Body100 {

    public Obstacle(Convex convex) {
        BodyFixture fixture = addFixture(convex, 1.0, 0.5, 0.5);
        fixture.setRestitutionVelocity(0.0);
        setMass(MassType.INFINITE);
        setAtRestDetectionEnabled(true);
    }

    @Override
    public void act() {
        // do nothing
    }
}

package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * There are many relatively short-lived notes, so they don't need
 * human-readable names.
 */
public class Note extends Body100 {
    private static int counter = 0;

    public Note(Convex convex) {
        super("note " + counter++);
        BodyFixture fixture = addFixture(convex, 1.0, 0.0, 0.0);
        // this means the springiness doesn't change with velocity
        fixture.setRestitutionVelocity(0.0);
        fixture.setFilter(NOTE);
        setMass(MassType.NORMAL);
    }

    @Override
    public void act() {
        // do nothing
    }

}

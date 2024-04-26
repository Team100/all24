package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * Slot sensor
 */
public class Speaker extends Body100 {
    // 1mm higher than the wall
    // TODO: some other way to keep the speaker and the wall from interfering
    private final Range m_vertical = new Range(1.984, 2.2);

    public Speaker(String id, Convex convex) {
        super(id);
        BodyFixture fixture = addFixture(convex, 1.0, 0.5, 0.5);
        fixture.setFilter(this);
        fixture.setSensor(true);
        setMass(MassType.INFINITE);
    }

    @Override
    protected Range getVerticalExtent() {
        return m_vertical;
    }
}

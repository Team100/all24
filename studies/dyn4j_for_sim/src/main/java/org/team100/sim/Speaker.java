package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * The lower extent of the speaker sensor is 1mm higher than the wall.
 * The upper extent is very high, modeling the ricochet inside the speaker.
 * 
 * The speaker is a sensor.
 */
public class Speaker extends Body100 {
    private final Range m_vertical = new Range(
            SimWorld.allianceWallHeightM + 0.001,
            4);

    public Speaker(String id, Convex convex) {
        super(id, false);
        BodyFixture fixture = addFixture(convex, 1.0, 0.5, 0.5);
        fixture.setFilter(this);
        fixture.setSensor(true);
        setMass(MassType.INFINITE);
    }

    @Override
    public Range getVerticalExtent() {
        return m_vertical;
    }
}

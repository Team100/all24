package org.team100.sim;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * Rectangular sensor, note this is not actually a dyn4j "Sensor" because we
 * want the note to bounce out if you shoot it too hard.
 */
public class AmpPocket extends Body100 {
    // 1mm further from the corner
    // TODO: some other way to keep the speaker and the wall from interfering
    // side walls are 0.508 so 0.660 is higher
    // TODO: remove the space in between
    private final Range m_vertical = new Range(0.660, 1.118);

    public AmpPocket(String id, Convex convex) {
        super(id);
        BodyFixture fixture = addFixture(convex, 1.0, 0.5, 0.5);
        fixture.setFilter(this);
        setMass(MassType.INFINITE);
    }

    @Override
    public Range getVerticalExtent() {
        return m_vertical;
    }
}

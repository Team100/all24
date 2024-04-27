package org.team100.sim;

import org.dyn4j.collision.Bounds;
import org.dyn4j.collision.CollisionBody;
import org.dyn4j.geometry.AABB;
import org.dyn4j.geometry.Polygon;
import org.dyn4j.geometry.Vector2;

/**
 * Bounds of the field, to detect notes to remove from the sim.
 */
public class Bounds100 implements Bounds {

    private final AABB m_boundsAABB;

    public Bounds100() {
        m_boundsAABB = new Polygon(
                new Vector2(0, 0),
                new Vector2(16.54, 0),
                new Vector2(16.54, 8.211),
                new Vector2(0, 8.211)).createAABB();
    }

    @Override
    public boolean isOutside(CollisionBody<?> body) {
        return !m_boundsAABB.overlaps(body.createAABB());
    }

    @Override
    public boolean isOutside(AABB aabb) {
        return !m_boundsAABB.overlaps(aabb);
    }
    /////////////////////////////////
    //
    // All the stuff below is unnecessary but it's part of the Bounds interface.

    @Override
    public void translate(double x, double y) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void translate(Vector2 vector) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void shift(Vector2 shift) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector2 getTranslation() {
        throw new UnsupportedOperationException();
    }

}

package org.team100.sim;

import org.dyn4j.collision.Filter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.TimeStep;
import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.PhysicsWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * There are many relatively short-lived notes, so they don't need
 * human-readable names.
 * 
 * Implements StepListener in order to update vertical motion.
 */
public class Note extends Body100 {
    private static final double kDrag = 0.3;
    private static final double kGravityM_s2 = -9.8;
    private static final double kMassKg = 0.235;

    private class NoteFilter extends FixedFilter {
        private boolean inert = false;

        @Override
        public boolean isAllowed(Filter filter) {
            if (inert)
                return false;
            return super.isAllowed(filter);
        }
    }

    private final NoteFilter m_filter;
    private static int counter = 0;
    private double m_altitude;
    private double m_verticalVelocityM_s;

    /** Don't forget to add the body as a step listener. */
    public Note() {
        super("note " + counter++);
        m_filter = new NoteFilter();
        // 14 inch diameter
        Circle geometry = Geometry.createCircle(0.175);
        // area is about 0.1 m^2. correct mass is 0.235 kg.
        // thus kg/m2 should be 2.35
        BodyFixture fixture = addFixture(geometry, 2.35, 1.0, 0.1);
        // this means the springiness doesn't change with velocity
        fixture.setRestitutionVelocity(0.0);
        fixture.setFilter(m_filter);
        setMass(MassType.NORMAL);
        m_altitude = 0;
        m_verticalVelocityM_s = 0;
        setInert(false);
        setFlying(false);
    }

    public void setFlying(boolean flying) {
        if (flying) {
            // the damping when in the air
            // spinning is very low drag
            setAngularDamping(0.1);
            // total guess?
            setLinearDamping(kDrag);
        } else {
            // the damping when on the floor
            setAngularDamping(10);
            setLinearDamping(5);
        }
    }

    /** Switch collisions on or off. */
    public void setInert(boolean inert) {
        m_filter.inert = inert;
    }

    /** For shooting. dyn4j handles x and y, and we handle z here. */
    public void applyImpulse(Translation3d impulseNs) {
        // apply 2d part
        Translation2d t = impulseNs.toTranslation2d();
        applyImpulse(new Vector2(t.getX(), t.getY()));
        // apply vertical part
        m_verticalVelocityM_s += impulseNs.getZ() / kMassKg;
        if (impulseNs.getZ() > 0) {
            setInert(true);
            setFlying(true);
        }
    }

    /**
     * Handles vertical motion.
     */
    @Override
    public void begin(TimeStep step, PhysicsWorld<Body100, ?> world) {
        if (m_altitude > 0 || m_verticalVelocityM_s > 0) {
            m_verticalVelocityM_s += step.getDeltaTime() * kGravityM_s2;
            // air drag
            double linear = 1.0 - step.getDeltaTime() * kDrag;
            m_verticalVelocityM_s *= linear;
            m_altitude += m_verticalVelocityM_s * step.getDeltaTime();
            if (m_altitude < 0) {
                m_altitude = 0;
                m_verticalVelocityM_s = 0;
                setInert(false);
                setFlying(false);
            }
        }
    }
}

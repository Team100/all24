package org.team100.sim;

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
    private static final double kAirDrag = 0.3;
    private static final double kGravityM_s2 = -9.8;
    private static final double kMassKg = 0.235;
    /** 2 inch thickness */
    private static final double kHeight = 0.05;
    /** 14 inch diameter */
    private static final double kDiameter = 0.175;
    /** While riding in a robot */
    private static final Range kCarriedRange = new Range(0.2, 0.25);

    private static int counter = 0;
    /** Altitude measures the bottom of the note. */
    private double m_altitude;
    private double m_verticalVelocityM_s;
    /** Is this note being carried by a robot? */
    private boolean m_carried;
    /** Used in calculation of vertical motion */
    private double preSpeed;

    /** prevent double counting */
    public boolean scored = false;

    /** Don't forget to add the body as a step listener. */
    public Note() {
        super("note " + counter++);
        Circle geometry = Geometry.createCircle(kDiameter);
        // area is about 0.1 m^2. correct mass is 0.235 kg.
        // thus kg/m2 should be 2.35
        BodyFixture fixture = addFixture(geometry, 2.35, 1.0, 0.1);
        // this means the springiness doesn't change with velocity
        fixture.setRestitutionVelocity(0.0);
        fixture.setFilter(this);
        setMass(MassType.NORMAL);
        m_altitude = 0;
        m_verticalVelocityM_s = 0;
        setFlying(false);
        setBullet(true);
    }

    public void setFlying(boolean flying) {
        if (flying) {
            // the damping when in the air
            // spinning is very low drag
            setAngularDamping(0.1);
            // total guess?
            setLinearDamping(kAirDrag);
        } else {
            // the damping when on the floor
            setAngularDamping(10);
            setLinearDamping(5);
        }
    }

    /** For shooting. dyn4j handles x and y, and we handle z here. */
    public void applyImpulse(Translation3d impulseNs) {
        // apply 2d part
        Translation2d t = impulseNs.toTranslation2d();
        applyImpulse(new Vector2(t.getX(), t.getY()));
        // apply vertical part
        m_verticalVelocityM_s += impulseNs.getZ() / kMassKg;
        if (impulseNs.getZ() > 0) {
            setFlying(true);
        }
    }

    /**
     * Handles vertical motion.
     */
    @Override
    public void begin(TimeStep step, PhysicsWorld<Body100, ?> world) {
        preSpeed = getLinearVelocity().getMagnitude();
    }

    /**
     * Handles vertical motion, using the ratio of 2d speeds to learn about drag and
     * collisions.
     */
    @Override
    public void end(TimeStep step, PhysicsWorld<Body100, ?> world) {
        // if the note is being carried, this doesn't do anything.
        if (m_carried)
            return;

        double postSpeed = getLinearVelocity().getMagnitude();

        if (m_altitude > 0 || m_verticalVelocityM_s > 0) {
            m_verticalVelocityM_s += step.getDeltaTime() * kGravityM_s2;
            // air drag and collisions
            double linear = postSpeed / preSpeed;
            m_verticalVelocityM_s *= linear;
            m_altitude += m_verticalVelocityM_s * step.getDeltaTime();
            if (m_altitude < 0) {
                m_altitude = 0;
                m_verticalVelocityM_s = 0;
                setFlying(false);
            }
        }

    }

    public void carry() {
        m_carried = true;
    }

    public void drop() {
        m_carried = false;
        m_altitude = 0;
    }

    @Override
    protected Range getVerticalExtent() {
        if (m_carried)
            return kCarriedRange;
        return new Range(m_altitude, m_altitude + kHeight);
    }
}

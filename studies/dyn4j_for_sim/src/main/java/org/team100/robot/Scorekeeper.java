package org.team100.robot;

import java.util.HashSet;
import java.util.Set;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.TimeStep;
import org.dyn4j.world.BroadphaseCollisionData;
import org.dyn4j.world.ManifoldCollisionData;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.listener.BoundsListener;
import org.dyn4j.world.listener.CollisionListener;
import org.dyn4j.world.listener.StepListener;
import org.team100.sim.AmpPocket;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.Speaker;

/**
 * Uses a CollisionListener to catch the collision event between notes and
 * speakers, and prints the updated score.
 * 
 * Removes bodies that leave the field -- this happens in the "end" step
 * listener method to avoid mutating the world at the wrong time.
 * 
 * TODO: add amp scoring
 * TODO: add amplification
 */
public class Scorekeeper
        implements CollisionListener<Body100, BodyFixture>,
        BoundsListener<Body100, BodyFixture>,
        StepListener<Body100> {

    private final Speaker m_blueSpeaker;
    private final Speaker m_redSpeaker;
    private final AmpPocket m_blueAmp;
    private final AmpPocket m_redAmp;
    private final Set<Body100> m_doomed;

    private int m_blueScore;
    private int m_redScore;

    public Scorekeeper(Speaker blue, Speaker red, AmpPocket blueAmp, AmpPocket redAmp) {
        m_blueSpeaker = blue;
        m_redSpeaker = red;
        m_blueAmp = blueAmp;
        m_redAmp = redAmp;
        m_doomed = new HashSet<>();
    }

    @Override
    public boolean collision(BroadphaseCollisionData<Body100, BodyFixture> collision) {
        return true;
    }

    /** Return true if the score is rejected (i.e. bounce out) */
    private boolean score(
            Body100 maybeNote,
            Body100 maybeSpeaker,
            Body100 sensor,
            Runnable handler,
            double maxSpeed) {
        if (maybeNote instanceof Note && maybeSpeaker == sensor) {

            Note n = (Note) maybeNote;
            if (n.scored) {
                // don't double count, don't eject after scoring
                return false;
            }
            // System.out.printf("sensor extent %s note extent %s\n",
            //         sensor.getVerticalExtent(),
            //         maybeNote.getVerticalExtent());
            if (!sensor.getVerticalExtent().contains(maybeNote.getVerticalExtent())) {
                // bounce out if note is not completely contained by the sensor.
                return true;
            }

            double speed = n.getLinearVelocity().getMagnitude();
            if (speed > maxSpeed) {
                // bounce out
                return true;
            }
            n.scored = true;
            // scored notes leave the field
            m_doomed.add(n);
            // System.out.printf("center %s altitude %5.3f\n", n.getWorldCenter(), n.getAltitude());
            handler.run();
            printScore();
            return false;
        }
        return true;
    }

    /** Return true if the score is rejected (i.e. bounce out). */
    private boolean tryScore(Body100 b1, Body100 b2, Body100 sensor, Runnable handler, double maxSpeed) {
        // false is success so if either one succeeds, we have success.
        return score(b1, b2, sensor, handler, maxSpeed) && score(b2, b1, sensor, handler, maxSpeed);
    }

    /**
     * Note, you can't use a filter to mimic the sensor behavior because the filter
     * is evaluated before the collision listener.
     */
    @Override
    public boolean collision(NarrowphaseCollisionData<Body100, BodyFixture> collision) {
        Body100 b1 = collision.getBody1();
        Body100 b2 = collision.getBody2();

        return tryScore(b1, b2, m_redSpeaker, () -> m_redScore++, Double.MAX_VALUE)
                && tryScore(b1, b2, m_blueSpeaker, () -> m_blueScore++, Double.MAX_VALUE)
                && tryScore(b1, b2, m_redAmp, () -> m_redScore++, 0.4)
                && tryScore(b1, b2, m_blueAmp, () -> m_blueScore++, 0.4);
    }

    private void printScore() {
        System.out.printf("Blue %d Red %d\n", m_blueScore, m_redScore);

    }

    @Override
    public boolean collision(ManifoldCollisionData<Body100, BodyFixture> collision) {
        return true;
    }

    @Override
    public void begin(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }

    @Override
    public void updatePerformed(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }

    @Override
    public void postSolve(TimeStep step, PhysicsWorld<Body100, ?> world) {
        //
    }

    @Override
    public void end(TimeStep step, PhysicsWorld<Body100, ?> world) {
        for (Body100 b : m_doomed) {
            world.removeBody(b);
        }
        m_doomed.clear();
    }

    /**
     * Removes objects that somehow made it outside.
     */
    @Override
    public void outside(Body100 body) {
        m_doomed.add(body);
    }

}

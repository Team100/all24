package org.team100.robot;

import java.util.LinkedList;
import java.util.List;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.TimeStep;
import org.dyn4j.world.BroadphaseCollisionData;
import org.dyn4j.world.ManifoldCollisionData;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.listener.BoundsListener;
import org.dyn4j.world.listener.CollisionListener;
import org.dyn4j.world.listener.StepListener;
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
    private final List<Body100> m_doomed;

    private int m_blueScore;
    private int m_redScore;

    public Scorekeeper(Speaker blue, Speaker red) {
        m_blueSpeaker = blue;
        m_redSpeaker = red;
        m_doomed = new LinkedList<>();
    }

    @Override
    public boolean collision(BroadphaseCollisionData<Body100, BodyFixture> collision) {
        return true;
    }

    @Override
    public boolean collision(NarrowphaseCollisionData<Body100, BodyFixture> collision) {
        Body100 b1 = collision.getBody1();
        Body100 b2 = collision.getBody2();
        if (b1 instanceof Note && b2 == m_redSpeaker) {
            Note n = (Note) b1;
            if (!n.scored) {
                n.scored = true;
                m_redScore++;
                printScore();
            }
        } else if (b1 == m_redSpeaker && b2 instanceof Note) {
            Note n = (Note) b2;
            if (!n.scored) {
                n.scored = true;
                m_redScore++;
                printScore();
            }
        } else if (b1 instanceof Note && b2 == m_blueSpeaker) {
            Note n = (Note) b1;
            if (!n.scored) {
                n.scored = true;
                m_blueScore++;
                printScore();
            }
        } else if (b1 == m_blueSpeaker && b2 instanceof Note) {
            Note n = (Note) b2;
            if (!n.scored) {
                n.scored = true;
                m_blueScore++;
                printScore();
            }
        }
        return true;
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

package org.team100.field;

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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Uses a CollisionListener to catch the collision event between notes and
 * speakers, and prints the updated score.
 * 
 * Removes bodies that leave the field -- this happens in the "end" step
 * listener method to avoid mutating the world at the wrong time.
 * 
 * TODO: add fouls
 */
public class Scorekeeper
        implements CollisionListener<Body100, BodyFixture>,
        BoundsListener<Body100, BodyFixture>,
        StepListener<Body100> {

    private final Speaker m_blueSpeaker;
    private final Speaker m_redSpeaker;
    private final AmpPocket m_blueAmp;
    private final AmpPocket m_redAmp;
    private final boolean m_debug;
    private final Set<Body100> m_doomed;

    private final Score m_blue;
    private final Score m_red;

    private Double m_blueAmpTime = null;
    private Double m_redAmpTime = null;
    private int m_blueAmplifiedCount = 0;
    private int m_redAmplifiedCount = 0;

    public Scorekeeper(
            Speaker blue,
            Speaker red,
            AmpPocket blueAmp,
            AmpPocket redAmp,
            boolean debug,
            Score blueScore,
            Score redScore) {
        m_blueSpeaker = blue;
        m_redSpeaker = red;
        m_blueAmp = blueAmp;
        m_redAmp = redAmp;
        m_debug = debug;
        m_blue = blueScore;
        m_red = redScore;
        m_doomed = new HashSet<>();
    }

    @Override
    public boolean collision(BroadphaseCollisionData<Body100, BodyFixture> collision) {
        return true;
    }

    /** Return true if the score is rejected (i.e. collision i.e. bounce out) */
    private boolean score(
            Body100 maybeNote,
            Body100 maybeSpeaker,
            Body100 sensor,
            Runnable handler,
            double maxSpeed) {
        if (maybeNote instanceof Note && maybeSpeaker == sensor) {

            Note note = (Note) maybeNote;
            if (note.scored) {
                // don't double count, don't eject after scoring
                return false;
            }
            if (m_debug)
                System.out.printf("sensor extent %s note extent %s\n",
                        sensor.getVerticalExtent(),
                        maybeNote.getVerticalExtent());
            if (!sensor.getVerticalExtent().contains(maybeNote.getVerticalExtent())) {
                if (m_debug) {
                    System.out.println("sensor does not contain note");
                }
                // bounce out if note is not completely contained by the sensor.
                return true;
            }

            double speed = note.getLinearVelocity().getMagnitude();
            if (speed > maxSpeed) {
                if (m_debug) {
                    System.out.println("Speed is higher than max");
                }
                // bounce out
                return true;
            }
            note.scored = true;
            // scored notes leave the field
            m_doomed.add(note);
            if (m_debug)
                System.out.printf("scored!  center %s altitude %5.3f\n", note.getWorldCenter(), note.getAltitude());
            handler.run();
            return false;
        }
        return true;
    }

    /**
     * Return true if the score is rejected (i.e. it's a "collision" so a bounce
     * out).
     */
    private boolean tryScore(
            Body100 b1,
            Body100 b2,
            Body100 sensor,
            Runnable handler,
            double maxSpeed) {
        // false is success so if either one succeeds, we have success.
        return score(b1, b2, sensor, handler, maxSpeed)
                && score(b2, b1, sensor, handler, maxSpeed);
    }

    /**
     * Note, you can't use a filter to mimic the sensor behavior because the filter
     * is evaluated before the collision listener.
     */
    @Override
    public boolean collision(NarrowphaseCollisionData<Body100, BodyFixture> collision) {
        Body100 b1 = collision.getBody1();
        Body100 b2 = collision.getBody2();

        return tryScore(b1, b2, m_redSpeaker, this::scoreRedSpeaker, Double.MAX_VALUE)
                && tryScore(b1, b2, m_blueSpeaker, this::scoreBlueSpeaker, Double.MAX_VALUE)
                && tryScore(b1, b2, m_redAmp, this::scoreRedAmp, 0.4)
                && tryScore(b1, b2, m_blueAmp, this::scoreBlueAmp, 0.4);
    }

    private void scoreBlueAmp() {
        if (DriverStation.isAutonomous()) {
            m_blue.AutoAmpNoteCount++;
        } else {
            m_blue.TeleopAmpNoteCount++;
            if (m_blue.TeleopAmpNoteCount - m_blueAmplifiedCount >= 2) {
                // time to amplify
                m_blueAmpTime = Timer.getFPGATimestamp();
                m_blueAmplifiedCount = m_blue.TeleopAmpNoteCount;
            }
        }
    }

    private void scoreRedAmp() {
        if (DriverStation.isAutonomous()) {
            m_red.AutoAmpNoteCount++;
        } else {
            m_red.TeleopAmpNoteCount++;
            if (m_red.TeleopAmpNoteCount - m_redAmplifiedCount >= 2) {
                // time to amplify
                m_redAmpTime = Timer.getFPGATimestamp();
                m_redAmplifiedCount = m_red.TeleopAmpNoteCount;
            }
        }
    }

    private void scoreBlueSpeaker() {
        if (DriverStation.isAutonomous()) {
            m_blue.AutoSpeakerNoteCount++;
        } else {
            if (m_blueAmpTime != null) {
                // amplified
                m_blue.TeleopSpeakerNoteCountAmplified++;
            } else {
                m_blue.TeleopSpeakerNoteCountNotAmplified++;
            }
        }
    }

    private void scoreRedSpeaker() {
        if (DriverStation.isAutonomous()) {
            m_red.AutoSpeakerNoteCount++;
        } else {
            if (m_redAmpTime != null) {
                // amplified
                m_red.TeleopSpeakerNoteCountAmplified++;
            } else {
                m_red.TeleopSpeakerNoteCountNotAmplified++;
            }
        }
    }

    public double redAmplified() {
        if (m_redAmpTime == null)
            return 0;
        double elapsedTime = Timer.getFPGATimestamp() - m_redAmpTime;
        double timeRemaining = 10 - elapsedTime;
        if (timeRemaining < 0) {
            m_redAmpTime = null;
            return 0;
        }
        return timeRemaining;
    }

    public double blueAmplified() {
        if (m_blueAmpTime == null)
            return 0;
        double elapsedTime = Timer.getFPGATimestamp() - m_blueAmpTime;
        double timeRemaining = 10 - elapsedTime;
        if (timeRemaining < 0) {
            m_blueAmpTime = null;
            return 0;
        }
        return timeRemaining;

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
        if (m_debug) {
            System.out.println("outside");
        }
        m_doomed.add(body);
    }

}

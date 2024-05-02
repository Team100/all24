package org.team100.field;

import java.util.Formatter;
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

import edu.wpi.first.wpilibj.Timer;

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

    private static final String kReset = "\033[0m";
    private static final String kBoldBlue = "\033[1;37;44m";
    private static final String kBlue = "\033[0;37;44m";
    private static final String kBoldRed = "\033[1;37;41m";
    private static final String kRed = "\033[0;37;41m";
    private static final String kAmped = "   Amplified!   ";
    private static final String kBlank = "                ";

    private final Speaker m_blueSpeaker;
    private final Speaker m_redSpeaker;
    private final AmpPocket m_blueAmp;
    private final AmpPocket m_redAmp;
    private final boolean m_debug;
    private final Set<Body100> m_doomed;
    private final Timer m_timer;

    private int m_blueScore;
    private int m_redScore;

    public Scorekeeper(
            Speaker blue,
            Speaker red,
            AmpPocket blueAmp,
            AmpPocket redAmp,
            boolean debug) {
        m_blueSpeaker = blue;
        m_redSpeaker = red;
        m_blueAmp = blueAmp;
        m_redAmp = redAmp;
        m_debug = debug;
        m_doomed = new HashSet<>();
        m_timer = new Timer();
        m_timer.start();
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
            if (m_debug)
                System.out.printf("sensor extent %s note extent %s\n",
                        sensor.getVerticalExtent(),
                        maybeNote.getVerticalExtent());
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
            if (m_debug)
                System.out.printf("center %s altitude %5.3f\n", n.getWorldCenter(), n.getAltitude());
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

    // to alternate amping
    private boolean amped = false;

    /** Like the audience display */
    public void newPrintScore() {
        // to work out the format
        double blueAmpTime = 7.5;
        double redAmpTime = 10.0;
        int fakeBlueScore = 63;
        int fakeRedScore = 82;
        double fakeMatchTimeSec = 82.532;
        int minutes = (int) fakeMatchTimeSec / 60;
        int seconds = (int) fakeMatchTimeSec % 60;
        StringBuilder b = new StringBuilder();
        Formatter f = new Formatter(b);
        if (amped) {
            b.append(kReset);
            f.format(" %2.0f ", blueAmpTime);
            b.append(kBoldBlue);
            b.append(kAmped);
        } else { // maintain alignment
            b.append(kReset);
            b.append("    "); // score placeholder
            b.append(kBlank);
        }
        b.append(kBlue);
        b.append(" Blue ");
        b.append(kBoldBlue);
        f.format(" %3d ", fakeBlueScore);
        b.append(kReset);
        f.format(" %2d:%02d ", minutes, seconds);
        b.append(kBoldRed);
        f.format(" %3d ", fakeRedScore);
        b.append(kRed);
        b.append(" Red  ");
        if (amped) {
            b.append(kBoldRed);
            b.append(kAmped);
            b.append(kReset);
            f.format(" %2.0f ", redAmpTime);
        }
        b.append(kReset);
        System.out.println(b.toString());
        f.close();
        amped ^= true;
    }

    /** Like the TBA summary, but with blue on the left. */
    public void printResults() {
        int ampPoint = 1;
        int notAmpedSpeakerPoint = 2;
        int ampedSpeakerPoint = 5;

        int redAmps = 7;
        int blueAmps = 7;
        int redNotAmpedSpeakers = 2;
        int redAmpedSpeakers = 10;
        int blueNotAmpedSpeakers = 10;
        int blueAmpedSpeakers = 4;

        StringBuilder b = new StringBuilder();
        Formatter f = new Formatter(b);

        b.append(kBlue);
        f.format("    %2d   ", blueAmps);
        b.append(kReset);
        b.append("   Teleop Amp Note Count   ");
        b.append(kRed);
        f.format("    %2d   ", redAmps);
        b.append(kReset);
        b.append("\n");

        b.append(kBlue);
        f.format(" %2d / %2d ", blueNotAmpedSpeakers, blueAmpedSpeakers);
        b.append(kReset);
        b.append(" Teleop Speaker Note Count ");
        b.append(kRed);
        f.format(" %2d / %2d ", redNotAmpedSpeakers, redAmpedSpeakers);
        b.append(kReset);
        b.append("\n");

        int blueTotal = blueAmps * ampPoint
                + blueNotAmpedSpeakers * notAmpedSpeakerPoint
                + blueAmpedSpeakers * ampedSpeakerPoint;
        int redTotal = redAmps * ampPoint
                + redNotAmpedSpeakers * ampedSpeakerPoint
                + redAmpedSpeakers * ampedSpeakerPoint;
        b.append(kBoldBlue);
        f.format("    %2d   ", blueTotal);
        b.append(kReset);
        b.append("     Teleop Note Points    ");
        b.append(kBoldRed);
        f.format("    %2d   ", redTotal);
        b.append(kReset);

        System.out.println(b.toString());
        f.close();
    }

    @Override
    public boolean collision(ManifoldCollisionData<Body100, BodyFixture> collision) {
        return true;
    }

    @Override
    public void begin(TimeStep step, PhysicsWorld<Body100, ?> world) {
        if (m_timer.advanceIfElapsed(1)){
            // just for testing the format
            newPrintScore();
            printResults();

        }
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

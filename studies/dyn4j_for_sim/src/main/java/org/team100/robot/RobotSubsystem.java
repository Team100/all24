package org.team100.robot;

import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Contains the sim body. */
public class RobotSubsystem extends SubsystemBase {

    public static record RobotSighting(boolean friend, Translation2d position) {
    }

    public static record NoteSighting(Translation2d position) {
    }

    /**
     * how old can sightings be and still be trusted?
     */
    private static final double kLookbackSec = 0.1;

    /**
     * Shooter impulse is measured in newton-seconds.
     * For a 0.235 kg note at 20 m/s the impulse is about 4.7 Ns.
     */
    private static final double kShootImpulseNs = 4.7;

    /**
     * Shooter angle is actually variable
     * TODO: implement shooter map
     */
    private static final double kShootElevationRad = -0.5;

    /**
     * Lob is slower, enough to get about 3.5m high and travel about 6m
     */
    private static final double kLobImpulseNs = 3.0;

    /**
     * Lob uses a pretty high angle, this is a guess.
     */
    private static final double kLobElevationRad = -1.0;

    /**
     * Amp shot is gentle
     */
    private static final double kAmpImpulseNs = 1.2;

    /**
     * Amp elevation is very high
     */
    private static final double kAmpElevationRad = -1.48;

    /** For simulation. */
    private final RobotBody m_robotBody;

    /**
     * We're allowed zero or one notes.
     * TODO: if the note is present, that should affect the sensor states.
     */
    private Note m_note;

    /** Joint linking the note to the robot, so we can remove it when ejecting. */
    private Joint<Body100> m_joint;

    /**
     * Some recent sightings from the camera system, used for robot avoidance and
     * defense.
     * 
     * We can't trust that the camera knows the identity of each sighting, just the
     * position (within some tolerance) and the time (quite precisely). We can also
     * detect friend-or-foe since the bumper color tells us. Key is time in sec.
     * Note: some time jitter should be used here since one camera frame may include
     * multiple sightings.
     */
    private NavigableMap<Double, RobotSighting> sightings = new ConcurrentSkipListMap<>();

    /** Recent note sightings. */
    private NavigableMap<Double, NoteSighting> noteSightings = new ConcurrentSkipListMap<>();

    public RobotSubsystem(RobotBody robotBody) {
        m_robotBody = robotBody;
    }

    public void intake() {
        // just one note allowed
        if (m_note != null)
            return;
        Vector2 position = m_robotBody.getWorldCenter();

        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (body instanceof Note) {
                Vector2 notePosition = body.getWorldCenter();
                double distance = position.distance(notePosition);
                if (distance > 0.3)
                    continue;
                // it's underneath the robot
                // TODO: intake from one side only
                m_note = (Note) body;
                // move the note to the center of the robot first
                m_note.setTransform(m_robotBody.getTransform());
                m_joint = new WeldJoint<>(m_note, m_robotBody, new Vector2());
                m_robotBody.getWorld().addJoint(m_joint);
                m_note.carry();
                break;
            }
        }
    }

    public RobotBody getRobotBody() {
        return m_robotBody;
    }

    /** meters and meters per second */
    public void setState(double x, double y, double vx, double vy) {
        m_robotBody.getTransform().identity();
        m_robotBody.getTransform().translate(x, y);
        m_robotBody.setAtRest(false);
        m_robotBody.setLinearVelocity(new Vector2(vx, vy));
    }

    /** Apply force and torque. Multiple calls to this method add. */
    public void apply(double x, double y, double theta) {
        m_robotBody.applyForce(new Force(x, y));
        m_robotBody.applyTorque(new Torque(theta));
    }

    public Pose2d getPose() {
        return m_robotBody.getPose();
    }

    public FieldRelativeVelocity getVelocity() {
        return m_robotBody.getVelocity();
    }

    @Override
    public void periodic() {
        lookForRobots();
        lookForNotes();
        trimSightings();
    }

    /**
     * In reality, each robot would get camera updates asynchronously (and
     * potentially out-of-order) but to keep it simple (and because dyn4j is not
     * thread safe), it's here.
     */
    private void lookForRobots() {
        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (body == m_robotBody) {
                // skip ourselves
                continue;
            }
            if (!(body instanceof RobotBody)) {
                // look only at robots
                continue;
            }
            RobotBody robotBody = (RobotBody) body;
            Vector2 targetPosition = robotBody.getWorldCenter();
            boolean friend = robotBody.friend(m_robotBody);
            addSighting(friend, targetPosition);
        }
    }

    /**
     * Camera looking for notes, sticking them in a buffer of note sightings. As
     * above this would actually be asynchronous.
     */
    private void lookForNotes() {
        Vector2 position = m_robotBody.getWorldCenter();
        // look for nearby notes, brute force
        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (body instanceof Note) {
                Vector2 notePosition = body.getWorldCenter();
                double distance = position.distance(notePosition);
                // can't see that far
                if (distance > 5)
                    continue;
                double now = Timer.getFPGATimestamp();
                NoteSighting sighting = new NoteSighting(
                        new Translation2d(notePosition.x, notePosition.y));
                noteSightings.put(now, sighting);
            }
        }
    }

    /** Add a sighting with the current timestamp. */
    private void addSighting(boolean friend, Vector2 fieldRelativePosition) {
        double now = Timer.getFPGATimestamp();
        RobotSighting sighting = new RobotSighting(
                friend,
                new Translation2d(fieldRelativePosition.x, fieldRelativePosition.y));
        sightings.put(now, sighting);
    }

    /** Don't remember stale sightings. */
    private void trimSightings() {
        double now = Timer.getFPGATimestamp();
        sightings.keySet().removeAll(sightings.headMap(now - kLookbackSec).keySet());
        noteSightings.keySet().removeAll(noteSightings.headMap(now - kLookbackSec).keySet());
    }

    public NavigableMap<Double, RobotSighting> recentSightings() {
        return sightings.descendingMap();
    }

    public Pose2d shootingPosition() {
        return m_robotBody.shootingPosition();
    }

    public Pose2d ampPosition() {
        return m_robotBody.ampPosition();
    }

    public Pose2d sourcePosition() {
        return m_robotBody.sourcePosition();
    }

    public Pose2d passingPosition() {
        return m_robotBody.passingPosition();
    }

    public void outtake() {
        if (m_note != null) {
            m_robotBody.getWorld().removeJoint(m_joint);
            m_note.drop();
            m_joint = null;
            m_note = null;
        }
    }

    /** Shooting is full speed, 20 m/s */
    public void shoot() {
        shooter(kShootImpulseNs, kShootElevationRad, 0);
    }

    /** Lob uses a lower exit velocity. */
    public void lob() {
        shooter(kLobImpulseNs, kLobElevationRad, 0);
    }

    /** Amp uses very low velocity, opposite rotation. */
    public void amp() {
        shooter(kAmpImpulseNs, kAmpElevationRad, Math.PI);
    }

    private void shooter(double impulseNs, double elevationRad, double yawOffsetRad) {
        if (m_note != null) {
            // detatch the note
            m_robotBody.getWorld().removeJoint(m_joint);
            m_note.drop();
            double yawRad = m_robotBody.getPose().getRotation().getRadians();
            yawRad = MathUtil.angleModulus(yawRad + yawOffsetRad);
            Translation3d t3 = new Translation3d(
                    impulseNs,
                    new Rotation3d(0, elevationRad, yawRad));
            m_note.applyImpulse(t3);
            m_joint = null;
            m_note = null;
        }
    }

}

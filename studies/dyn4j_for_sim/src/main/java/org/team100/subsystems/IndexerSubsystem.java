package org.team100.subsystems;

import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Vector2;
import org.team100.robot.RobotAssembly;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Manages the note and joint. */
public class IndexerSubsystem extends SubsystemBase {

    /**
     * Intake velocity admittance: need to drive over the note.
     */
    private static final double kVelocityAdmittanceRad = Math.PI / 4;
    /**
     * Intake admittance half-angle: need to approach from the intake side.
     */
    public static final double kAdmittanceRad = 0.2;
    private final RobotAssembly m_assembly;
    private final RobotBody m_robotBody;
    private final boolean m_debug;

    /**
     * We're allowed zero or one notes.
     * TODO: if the note is present, that should affect the sensor states.
     */
    private Note m_note;

    /** Joint linking the note to the robot, so we can remove it when ejecting. */
    private Joint<Body100> m_joint;

    public IndexerSubsystem(RobotAssembly assembly, RobotBody robotBody, boolean debug) {
        m_assembly = assembly;
        m_robotBody = robotBody;
        m_debug = debug;
    }

    // public boolean aligned(double angleRad) {
    // return Math.abs(angleRad) < IndexerSubsystem.kAdmittanceRad;
    // }

    /** There's a note in the indexer */
    public boolean full() {
        return m_note != null;
    }

    /**
     * Puts the note in the indexer.
     * 
     * Returns false if the indexer is already full.
     */
    public boolean intake() {
        // Correct center-to-center distance for picking.
        final double kMinPickDistanceM = 0.437;

        if (m_note != null) {
            return false;
        }

        Vector2 position = m_robotBody.getWorldCenter();

        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (!(body instanceof Note)) {
                // pick only notes
                continue;
            }
            Note note = (Note) body;

            if (note.isFlying()) {
                // do not pick from mid-air.
                continue;
            }

            Vector2 notePosition = note.getWorldCenter();
            Vector2 toNote = notePosition.difference(position);
            double distance = toNote.getMagnitude();
            if (distance < kMinPickDistanceM || distance > 0.488) {
                // distance must be within an inch or so of the intake touching the note edge.
                // robot size is 0.75, note size is 0.175.
                // so the tangent distance is about (0.75/2+0.175/2) = 0.4625.
                // so a reasonable range is 0.437-0.488
                continue;
            }

            double angleToNote = toNote.getDirection();
            double intakeAngle = m_robotBody.getRotationAngle() + Math.PI;
            double angleError = MathUtil.angleModulus(angleToNote - intakeAngle);
            if (Math.abs(angleError) >= kAdmittanceRad) {
                // to pick, the note needs to aligned to the correct side.
                continue;
            }

            Vector2 robotVelocity = m_robotBody.getLinearVelocity();
            double velocityAngle = toNote.getAngleBetween(robotVelocity);
            if (Math.abs(velocityAngle) > kVelocityAdmittanceRad) {
                // to pick, the robot needs to be moving towards it.
                continue;
            }

            // successful pick.
            load(note);
            return true;
        }
        return false;
    }

    public void load(Note note) {
        m_note = note;
        // move the note to the center of the robot
        m_note.setTransform(m_robotBody.getTransform());
        m_joint = new WeldJoint<>(m_note, m_robotBody, new Vector2());
        m_robotBody.getWorld().addJoint(m_joint);
        m_note.carry();
    }

    public void preload() {
        Note note = new Note(m_debug);
        SimWorld world = m_robotBody.getWorld();
        world.addBody(note);
        world.addStepListener(note);
        load(note);
    }

    /**
     * Ejects the note.
     * 
     * Returns false if there is no note to eject.
     */
    public boolean outtake() {
        if (m_note == null) {
            return false;
        }
        m_robotBody.getWorld().removeJoint(m_joint);
        m_note.drop();
        m_joint = null;
        m_note = null;
        return true;
    }

    /**
     * Removes and returns the note from the indexer and stashes it in the assembly
     * handoff.
     * 
     * Returns false if empty.
     */
    public boolean towardsShooter() {
        if (m_note == null) {
            return false;
        }
        m_robotBody.getWorld().removeJoint(m_joint);
        m_note.drop();
        m_assembly.m_indexerShooterHandoff = m_note;
        m_joint = null;
        m_note = null;
        return true;
    }
}

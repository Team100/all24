package org.team100.robot;

import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Vector2;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manages the note and joint
 */
public class IndexerSubsystem extends SubsystemBase {
    private final RobotBody m_robotBody;

    /**
     * We're allowed zero or one notes.
     * TODO: if the note is present, that should affect the sensor states.
     */
    private Note m_note;

    /** Joint linking the note to the robot, so we can remove it when ejecting. */
    private Joint<Body100> m_joint;

    public IndexerSubsystem(RobotBody robotBody) {
        m_robotBody = robotBody;
    }

    /**
     * Puts the note in the indexer.
     * 
     * Returns false if the indexer is already full.
     */
    public boolean offer(Note note) {
        if (m_note != null)
            return false;
        m_note = note;
        // move the note to the center of the robot first
        m_note.setTransform(m_robotBody.getTransform());
        m_joint = new WeldJoint<>(m_note, m_robotBody, new Vector2());
        m_robotBody.getWorld().addJoint(m_joint);
        m_note.carry();
        return true;
    }

    /**
     * Removes and returns the note from the indexer.
     * 
     * Returns null if empty.
     */
    public Note poll() {
        m_robotBody.getWorld().removeJoint(m_joint);
        m_note.drop();
        Note result = m_note;
        m_joint = null;
        m_note = null;
        return result;
    }
}

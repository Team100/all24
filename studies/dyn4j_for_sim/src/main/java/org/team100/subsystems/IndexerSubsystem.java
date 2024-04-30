package org.team100.subsystems;

import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Vector2;
import org.team100.robot.RobotAssembly;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Manages the note and joint. */
public class IndexerSubsystem extends SubsystemBase {
    private final RobotAssembly m_assembly;
    private final RobotBody m_robotBody;

    /**
     * We're allowed zero or one notes.
     * TODO: if the note is present, that should affect the sensor states.
     */
    private Note m_note;

    /** Joint linking the note to the robot, so we can remove it when ejecting. */
    private Joint<Body100> m_joint;

    public IndexerSubsystem(RobotAssembly assembly, RobotBody robotBody) {
        m_assembly = assembly;
        m_robotBody = robotBody;
    }

    /** There's a note in the indexer */
    public boolean full() {
        System.out.println("full: " + (m_note != null));
        return m_note != null;
    }

    /**
     * Puts the note in the indexer.
     * 
     * Returns false if the indexer is already full.
     */
    public boolean intake() {
        if (m_note != null) {
            return false;
        }

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

                if (m_note.isFlying())
                    continue;

                // move the note to the center of the robot first
                m_note.setTransform(m_robotBody.getTransform());
                m_joint = new WeldJoint<>(m_note, m_robotBody, new Vector2());
                m_robotBody.getWorld().addJoint(m_joint);
                m_note.carry();
                return true;
            }
        }
        return false;
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

package org.team100.robot;

import org.team100.commands.ShootCommand;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Contains the subsystems of a single robot, and manages simulation handoffs.
 * The handoffs are managed in this "hidden" way because the identity of the
 * handed-off things are invisible to the real robot, so it doesn't make sense
 * for "real" robot commands to know about them.
 */
public class RobotAssembly {

    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;

    /*
     * contains a note if the indexer has ejected it towards the shooter, but the
     * shooter hasn't yet picked it up.
     */
    public Note m_indexerShooterHandoff;

    public RobotAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        m_indexer = new IndexerSubsystem(this, robotBody);
        m_shooter = new ShooterSubsystem(this, robotBody, speakerPosition);

        new Trigger(() -> false).onTrue(new ShootCommand(m_indexer, m_shooter));
    }

}

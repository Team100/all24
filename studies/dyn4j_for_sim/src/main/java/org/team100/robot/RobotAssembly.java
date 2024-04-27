package org.team100.robot;

import org.team100.commands.ShootCommand;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Contains the subsystems of a single robot. */
public class RobotAssembly {

    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;

    public RobotAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        m_indexer = new IndexerSubsystem(robotBody);
        m_shooter = new ShooterSubsystem(robotBody, speakerPosition);

        new Trigger(() -> false).onTrue(new ShootCommand(m_indexer, m_shooter));
    }

}

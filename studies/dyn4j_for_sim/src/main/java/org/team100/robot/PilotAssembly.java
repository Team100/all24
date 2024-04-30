package org.team100.robot;

import java.util.function.BooleanSupplier;

import org.team100.commands.DriveToSource;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.Intake;
import org.team100.commands.PilotDrive;
import org.team100.commands.RotateToShoot;
import org.team100.commands.ShootCommand;
import org.team100.control.Pilot;
import org.team100.control.auto.Autopilot;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An attempt to make an assembly that runs like the player one does, but with
 * an autopilot.
 */
public class PilotAssembly extends RobotAssembly {

    public PilotAssembly(Pilot pilot, RobotBody robotBody, Translation2d speakerPosition) {
        super(robotBody, speakerPosition);
        m_drive.setDefaultCommand(new PilotDrive(m_drive, pilot));
        // one way to do sequences is via Command.andThen().
        whileTrue(pilot::driveToSpeaker,
                new DriveToSpeaker(null, this)
                        .andThen(new RotateToShoot(speakerPosition, m_drive))
                        .andThen(new ShootCommand(m_indexer, m_shooter))
                        .finallyDo(pilot::onEnd));
        // TODO: add drive-to-note
        whileTrue(pilot::driveToSource,
                // new PrintCommand("asdf"));
                Commands.deadline(
                        new Intake(m_indexer),
                        new DriveToSource(null, this))
                        .finallyDo(pilot::onEnd));
    }

    ///////////////////////////////////////////

    private void whileTrue(BooleanSupplier condition, Command command) {
        // note that triggers detect edges, not states
        new Trigger(condition).whileTrue(command);
    }

}

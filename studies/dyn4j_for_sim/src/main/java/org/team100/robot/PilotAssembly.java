package org.team100.robot;

import org.team100.commands.AmpCommand;
import org.team100.commands.DefendSource;
import org.team100.commands.DriveToAmp;
import org.team100.commands.DriveToPass;
import org.team100.commands.DriveToSource;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.Intake;
import org.team100.commands.LobCommand;
import org.team100.commands.PilotDrive;
import org.team100.commands.RotateToShoot;
import org.team100.commands.ShootCommand;
import org.team100.control.Pilot;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;

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
                new DriveToSpeaker(m_drive, m_camera, m_drive.shootingPosition())
                        .andThen(new RotateToShoot(speakerPosition, m_drive))
                        .andThen(new ShootCommand(m_indexer, m_shooter))
                        .finallyDo(pilot::onEnd));
        // TODO: add drive-to-note
        whileTrue(pilot::driveToSource,
                Commands.deadline(
                        new Intake(m_indexer),
                        new DriveToSource(m_drive, m_camera, m_drive.sourcePosition()))
                        .finallyDo(pilot::onEnd));
        whileTrue(pilot::driveToAmp,
                new DriveToAmp(m_drive, m_camera, m_drive.ampPosition())
                        .andThen(new AmpCommand(m_indexer, m_shooter))
                        .finallyDo(pilot::onEnd));
        whileTrue(pilot::driveToPass,
                new DriveToPass(m_drive, m_camera, m_drive.passingPosition())
                        .andThen(new LobCommand(m_indexer, m_shooter))
                        .finallyDo(pilot::onEnd));
        // defend never ends
        whileTrue(pilot::defend,
                new DefendSource(m_drive, m_camera)
                        .finallyDo(pilot::onEnd));
    }
}

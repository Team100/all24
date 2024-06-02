package org.team100.robot;

import java.util.function.Function;

import org.team100.commands.AmpCommand;
import org.team100.commands.DefendSource;
import org.team100.commands.DriveToAmp;
import org.team100.commands.DriveToNote;
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

    public PilotAssembly(
            Function<RobotAssembly, Pilot> pilotFn,
            RobotBody robotBody,
            Translation2d speakerPosition,
            boolean debug) {
        super(pilotFn, robotBody, speakerPosition);

        m_drive.setDefaultCommand(new PilotDrive(m_drive, m_pilot));

        whileTrue(m_pilot::intake,
                new Intake(m_indexer));
        whileTrue(m_pilot::driveToSpeaker,
                Commands.sequence(
                        new DriveToSpeaker(m_drive, m_camera, m_drive.shootingPosition(), debug),
                        new RotateToShoot(speakerPosition, m_drive, debug),
                        new ShootCommand(m_indexer, m_shooter, debug)));
        whileTrue(m_pilot::driveToSource,
                new DriveToSource(m_drive, m_camera, m_drive.sourcePosition(), debug));
        whileTrue(m_pilot::driveToAmp,
                Commands.sequence(
                        new DriveToAmp(m_drive, m_camera, m_drive.ampPosition(), debug),
                        new AmpCommand(m_indexer, m_shooter)));
        whileTrue(m_pilot::driveToPass,
                Commands.sequence(
                        new DriveToPass(m_drive, m_camera, m_drive.passingPosition(), debug),
                        new LobCommand(m_indexer, m_shooter)));
        whileTrue(m_pilot::driveToNote,
                new DriveToNote(m_drive, m_camera, debug));
        whileTrue(m_pilot::defend,
                new DefendSource(0.1, m_drive, m_camera, debug));
    }
}

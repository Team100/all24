package org.team100.robot;

import java.util.function.Function;

import org.team100.commands.AmpCommand;
import org.team100.commands.DriveToAmp;
import org.team100.commands.DriveToPass;
import org.team100.commands.DriveToSource;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.Intake;
import org.team100.commands.LobCommand;
import org.team100.commands.Outtake;
import org.team100.commands.PilotDrive;
import org.team100.commands.RotateToShoot;
import org.team100.commands.ShootCommand;
import org.team100.control.Pilot;
import org.team100.sim.RobotBody;

/** This robot is controlled by a human. */
public class RealPlayerAssembly extends RobotAssembly {

    public RealPlayerAssembly(
            Function<RobotAssembly, Pilot> pilotFn,
            RobotBody robotBody,
            boolean debug) {
        super(pilotFn, robotBody, debug);

        // manual drive control
        m_drive.setDefaultCommand(new PilotDrive(m_drive, m_pilot));

        // buttons and commands are 1:1
        whileTrue(m_pilot::intake,
                new Intake(m_indexer));
        whileTrue(m_pilot::outtake,
                new Outtake(m_indexer));
        whileTrue(m_pilot::shoot,
                new ShootCommand(m_indexer, m_shooter, debug));
        whileTrue(m_pilot::lob,
                new LobCommand(m_indexer, m_shooter));
        whileTrue(m_pilot::amp,
                new AmpCommand(m_indexer, m_shooter));
        whileTrue(m_pilot::rotateToShoot,
                new RotateToShoot(m_drive, debug));
        whileTrue(m_pilot::driveToSpeaker,
                new DriveToSpeaker(m_drive, m_camera, debug));
        whileTrue(m_pilot::driveToAmp,
                new DriveToAmp(m_drive, m_camera, debug));
        whileTrue(m_pilot::driveToSource,
                new DriveToSource(m_drive, m_camera, debug));
        whileTrue(m_pilot::driveToPass,
                new DriveToPass(m_drive, m_camera, debug));
        whileTrue(m_pilot::shootCommand,
                new ShootCommand(m_indexer, m_shooter, debug));
    }
}

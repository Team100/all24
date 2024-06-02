package org.team100.robot;

import java.util.function.Function;

import org.team100.commands.DriveToAmp;
import org.team100.commands.DriveToPass;
import org.team100.commands.DriveToSource;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.Intake;
import org.team100.commands.PilotDrive;
import org.team100.commands.RotateToShoot;
import org.team100.commands.ShootCommand;
import org.team100.control.Pilot;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;

/** This robot is controlled by a human. */
public class RealPlayerAssembly extends RobotAssembly {

    public RealPlayerAssembly(
            Function<RobotAssembly, Pilot> pilotFn,
            RobotBody robotBody,
            Translation2d speakerPosition) {
        super(pilotFn, robotBody, speakerPosition);

        m_drive.setDefaultCommand(new PilotDrive(m_drive, m_pilot));

        whileTrue(m_pilot::intake,
                new Intake(m_indexer));
        whileTrue(m_pilot::outtake,
                m_indexer.run(m_indexer::outtake));
        whileTrue(m_pilot::shoot,
                Commands.parallel(
                        m_indexer.run(m_indexer::towardsShooter),
                        m_shooter.run(m_shooter::shoot)));
        whileTrue(m_pilot::lob,
                Commands.parallel(
                        m_indexer.run(m_indexer::towardsShooter),
                        m_shooter.run(m_shooter::lob)));
        whileTrue(m_pilot::amp,
                Commands.parallel(
                        m_indexer.run(m_indexer::towardsShooter),
                        m_shooter.run(m_shooter::amp)));
        whileTrue(m_pilot::rotateToShoot,
                new RotateToShoot(speakerPosition, m_drive, false));
        whileTrue(m_pilot::driveToSpeaker,
                new DriveToSpeaker(m_drive, m_camera, m_drive.shootingPosition(), false));
        whileTrue(m_pilot::driveToAmp,
                new DriveToAmp(m_drive, m_camera, m_drive.ampPosition(), false));
        whileTrue(m_pilot::driveToSource,
                new DriveToSource(m_drive, m_camera, m_drive.sourcePosition(), false));
        whileTrue(m_pilot::driveToPass,
                new DriveToPass(m_drive, m_camera, m_drive.passingPosition(), false));
        whileTrue(m_pilot::shootCommand,
                new ShootCommand(m_indexer, m_shooter, false));
    }
}

package org.team100.robot;

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

    private final Pilot m_control;

    public RealPlayerAssembly(Pilot pilot, RobotBody robotBody, Translation2d speakerPosition) {
        super(robotBody, speakerPosition);
        m_control = pilot;
        m_drive.setDefaultCommand(new PilotDrive(m_drive, m_control));

        whileTrue(m_control::intake,
                new Intake(m_indexer)
                        .finallyDo(() -> System.out.println("done intaking")));
        whileTrue(m_control::outtake,
                m_indexer.run(m_indexer::outtake));
        whileTrue(m_control::shoot,
                Commands.parallel(
                        m_indexer.run(m_indexer::towardsShooter),
                        m_shooter.run(m_shooter::shoot)));
        whileTrue(m_control::lob,
                Commands.parallel(
                        m_indexer.run(m_indexer::towardsShooter),
                        m_shooter.run(m_shooter::lob)));
        whileTrue(m_control::amp,
                Commands.parallel(
                        m_indexer.run(m_indexer::towardsShooter),
                        m_shooter.run(m_shooter::amp)));
        whileTrue(m_control::rotateToShoot,
                new RotateToShoot(speakerPosition, m_drive, false)
                        .finallyDo(x -> System.out.println("done rotating" + x)));
        whileTrue(m_control::driveToSpeaker,
                new DriveToSpeaker(m_drive, m_camera, m_drive.shootingPosition(), false)
                        .finallyDo(x -> System.out.println("done driving " + x)));
        whileTrue(m_control::driveToAmp,
                new DriveToAmp(m_drive, m_camera, m_drive.ampPosition(), false)
                        .finallyDo(x -> System.out.println("done driving " + x)));
        whileTrue(m_control::driveToSource,
                new DriveToSource(m_drive, m_camera, m_drive.sourcePosition(), false)
                        .finallyDo(x -> System.out.println("done driving " + x)));
        whileTrue(m_control::driveToPass,
                new DriveToPass(m_drive, m_camera, m_drive.passingPosition(), false)
                        .finallyDo(x -> System.out.println("done driving " + x)));
        whileTrue(m_control::shootCommand,
                new ShootCommand(m_indexer, m_shooter, false)
                        .finallyDo(x -> System.out.println("done shooting " + x)));
    }
}

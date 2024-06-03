package org.team100.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;
import org.team100.commands.AmpCommand;
import org.team100.commands.DefendSource;
import org.team100.commands.DriveToAmp;
import org.team100.commands.DriveToNote;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Contains the subsystems of a single robot, and manages simulation handoffs.
 * The handoffs are managed in this "hidden" way because the identity of the
 * handed-off things are invisible to the real robot, so it doesn't make sense
 * for "real" robot commands to know about them.
 */
public class RobotAssembly {
    protected final Pilot m_pilot;
    protected final DriveSubsystem m_drive;
    protected final IndexerSubsystem m_indexer;
    protected final ShooterSubsystem m_shooter;
    protected final CameraSubsystem m_camera;

    /*
     * Contains a note if the indexer has ejected it towards the shooter, but the
     * shooter hasn't yet picked it up.
     */
    public Note m_indexerShooterHandoff;

    public RobotAssembly(
            Function<RobotAssembly, Pilot> pilotFn,
            RobotBody robotBody,
            boolean debug) {
        m_drive = new DriveSubsystem(robotBody);
        m_indexer = new IndexerSubsystem(this, robotBody);
        m_shooter = new ShooterSubsystem(this, robotBody, debug);
        m_camera = new CameraSubsystem(robotBody);
        // must come after the initializations above.
        m_pilot = pilotFn.apply(this);

        // manual drive control
        m_drive.setDefaultCommand(new PilotDrive(m_drive, m_pilot));

        /////////////////////////////////////////////////////
        //
        // things that both the driver and autopilot would do

        whileTrue(m_pilot::intake,
                new Intake(m_indexer));
        whileTrue(m_pilot::defend,
                new DefendSource(0.1, m_drive, m_camera, debug));
        whileTrue(m_pilot::driveToNote,
                new DriveToNote(m_drive, m_camera, debug));
        whileTrue(m_pilot::driveToSource,
                new DriveToSource(m_drive, m_camera, debug));
        whileTrue(m_pilot::scoreSpeaker,
                Commands.sequence(
                        new DriveToSpeaker(m_drive, m_camera, debug),
                        new RotateToShoot(m_drive, debug),
                        new ShootCommand(m_indexer, m_shooter, debug)));
        whileTrue(m_pilot::scoreAmp,
                Commands.sequence(
                        new DriveToAmp(m_drive, m_camera, debug),
                        new AmpCommand(m_indexer, m_shooter)));
        whileTrue(m_pilot::pass,
                Commands.sequence(
                        new DriveToPass(m_drive, m_camera, debug),
                        new LobCommand(m_indexer, m_shooter)));

        ///////////////////////////////////////////////////////////////
        //
        // things that a driver might do but the autopilot would not do

        // the autopilot never needs to outtake
        whileTrue(m_pilot::outtake, new Outtake(m_indexer));

        // these are all parts of auto sequences; the autopilot never does them in
        // isolation.
        whileTrue(m_pilot::shoot,
                new ShootCommand(m_indexer, m_shooter, debug));
        whileTrue(m_pilot::amp,
                new AmpCommand(m_indexer, m_shooter));
        whileTrue(m_pilot::lob,
                new LobCommand(m_indexer, m_shooter));
        whileTrue(m_pilot::rotateToShoot,
                new RotateToShoot(m_drive, debug));
        whileTrue(m_pilot::shootCommand,
                new ShootCommand(m_indexer, m_shooter, debug));

    }

    public void setState(double x, double y, double vx, double vy) {
        m_drive.setState(x, y, vx, vy);
    }

    public DriveSubsystem getDrive() {
        return m_drive;
    }

    public CameraSubsystem getCamera() {
        return m_camera;
    }

    public IndexerSubsystem getIndexer() {
        return m_indexer;
    }

    public void reset() {
        m_pilot.reset();
    }

    public void begin() {
        m_pilot.begin();
    }

    public void periodic() {
        m_pilot.periodic();
    }

    protected void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }
}

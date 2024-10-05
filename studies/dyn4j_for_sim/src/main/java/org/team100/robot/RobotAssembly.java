package org.team100.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import org.team100.commands.AmpCommand;
import org.team100.commands.Intake;
import org.team100.commands.LobCommand;
import org.team100.commands.Outtake;
import org.team100.commands.PilotDrive;
import org.team100.commands.RotateToShoot;
import org.team100.commands.ShootCommand;
import org.team100.lib.commands.semiauto.DefendSource;
import org.team100.lib.commands.semiauto.DriveToNote;
import org.team100.lib.commands.semiauto.DriveToPose;
import org.team100.lib.commands.semiauto.DriveToSource;
import org.team100.lib.commands.semiauto.GoToStaged;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.pilot.Pilot;
import org.team100.lib.planner.ForceViz;
import org.team100.lib.planner.Tactics;
import org.team100.lib.util.Tolerance;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;

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
            SwerveKinodynamics swerveKinodynamics,
            Function<RobotAssembly, Pilot> pilotFn,
            RobotBody robotBody,
            ForceViz viz,
            boolean debug) {
        m_drive = new DriveSubsystem(robotBody, debug);
        m_indexer = new IndexerSubsystem(this, robotBody, debug);
        // every robot gets a preload in the indexer.
        m_indexer.preload();
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
                new Intake(m_indexer, debug));
        whileTrue(m_pilot::defend,
                new DefendSource(
                        swerveKinodynamics,
                        0.1,
                        m_drive,
                        m_camera::recentSightings,
                        robotBody::defenderPosition,
                        robotBody::opponentSourcePosition,
                        new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, false, true,
                                false, debug),
                        viz,
                        debug));
        whileTrue(m_pilot::driveToNote,
                new DriveToNote(
                        swerveKinodynamics,
                        m_drive,
                        m_camera::findClosestNoteTranslation,
                        new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true, true,
                                true, debug),
                        viz,
                        debug));
        whileTrue(m_pilot::driveToCorner,
                new DriveToPose(
                        swerveKinodynamics,
                        m_drive,
                        m_pilot::cornerLocation,
                        () -> 0.0,
                        new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true, true,
                                true, debug),
                        new Tolerance(1, 1, 0.25),
                        viz,
                        debug));
        whileTrue(m_pilot::driveToSource,
                new DriveToSource(
                        swerveKinodynamics,
                        m_drive,
                        robotBody::sourcePosition,
                        robotBody::yBias,
                        new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true, true,
                                true, debug),
                        viz,
                        debug));
        whileTrue(m_pilot::driveToStaged,
                new GoToStaged(
                        swerveKinodynamics,
                        m_pilot,
                        m_drive,
                        new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true, true,
                                true, debug),
                        viz,
                        debug));
        whileTrue(m_pilot::scoreSpeaker,
                Commands.sequence(
                        // here the lane accuracy issue is no problem
                        new DriveToPose(
                                swerveKinodynamics,
                                m_drive,
                                m_pilot::shootingLocation,
                                robotBody::yBias,
                                new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true,
                                        true, true,
                                        debug),
                                new Tolerance(1, 1, 0.25),
                                viz,
                                debug),
                        // rotation takes care of cartesian error.
                        new RotateToShoot(
                                m_drive,
                                robotBody::speakerPosition,
                                new Tolerance(1, 0.05, 0.05),
                                debug),
                        new ShootCommand(m_indexer, m_shooter, debug)));
        whileTrue(m_pilot::scoreAmp,
                Commands.sequence(
                        // first go approximately there, in the "lane"
                        new DriveToPose(
                                swerveKinodynamics,
                                m_drive,
                                robotBody::ampPosition,
                                robotBody::yBias,
                                new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true,
                                        false, true,
                                        debug),
                                new Tolerance(0.5, 0.5, 0.5),
                                viz,
                                debug),
                        // then go exactly there, position is important
                        new DriveToPose(
                                swerveKinodynamics,
                                m_drive,
                                robotBody::ampPosition,
                                () -> 0.0,
                                new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, false,
                                        false, false,
                                        debug),
                                new Tolerance(0.05, 0.05, 0.05),
                                viz,
                                debug),
                        new AmpCommand(m_indexer, m_shooter, debug)));
        whileTrue(m_pilot::pass,
                Commands.sequence(
                        // location can be pretty approximate
                        new DriveToPose(
                                swerveKinodynamics,
                                m_drive,
                                robotBody::passingPosition,
                                () -> 0.0,
                                new Tactics(swerveKinodynamics, m_drive::getPose, m_camera::recentSightings, viz, true,
                                        true, true,
                                        debug),
                                new Tolerance(0.3, 0.3, 0.1),
                                viz,
                                debug),
                        new LobCommand(m_indexer, m_shooter, debug)));

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
                new AmpCommand(m_indexer, m_shooter, debug));
        whileTrue(m_pilot::lob,
                new LobCommand(m_indexer, m_shooter, debug));
        whileTrue(m_pilot::rotateToShoot,
                new RotateToShoot(
                        m_drive,
                        robotBody::speakerPosition,
                        new Tolerance(1, 0.05, 0.05),
                        debug));
        whileTrue(m_pilot::shootCommand,
                new ShootCommand(m_indexer, m_shooter, debug));

    }

    /** Coordinates here use "blue" origin. */
    public void setState(double x, double y, double theta, double vx, double vy) {
        m_drive.setState(x, y, theta, vx, vy);
    }

    public DriveSubsystemInterface getDrive() {
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

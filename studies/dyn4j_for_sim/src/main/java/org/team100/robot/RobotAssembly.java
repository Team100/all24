package org.team100.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;
import org.team100.control.Pilot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Contains the subsystems of a single robot, and manages simulation handoffs.
 * The handoffs are managed in this "hidden" way because the identity of the
 * handed-off things are invisible to the real robot, so it doesn't make sense
 * for "real" robot commands to know about them.
 */
public abstract class RobotAssembly {
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

    protected RobotAssembly(
            Function<RobotAssembly, Pilot> pilotFn,
            RobotBody robotBody,
            boolean debug) {
        m_drive = new DriveSubsystem(robotBody);
        m_indexer = new IndexerSubsystem(this, robotBody);
        m_shooter = new ShooterSubsystem(this, robotBody, debug);
        m_camera = new CameraSubsystem(robotBody);
        // must come after the initializations above.
        m_pilot = pilotFn.apply(this);
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

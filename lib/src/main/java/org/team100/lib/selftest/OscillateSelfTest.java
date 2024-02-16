package org.team100.lib.selftest;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Test the Oscillate command. */
@ExcludeFromJacocoGeneratedReport
public class OscillateSelfTest extends Command {
    private static final double kExpectedDuration = 10;
    // abort if the robot moves too far
    private static final double kMaxDistance = 1.5;

    private final SwerveDriveSubsystem m_drivetrain;
    private final SelfTestListener m_listener;
    private final Timer m_timer;
    private final boolean m_direct;
    private final boolean m_rotation;
    private boolean terminate = false;
    private boolean prevOscillateTheta;
    private boolean prevOscillateDirect;
    private Pose2d m_initial;

    /**
     * 
     * @param drivetrain
     * @param listener
     * @param direct     use direct module state control. otherwise use chassis
     *                   speeds.
     * @param rotation   drive theta. otherwise drive x.
     */
    public OscillateSelfTest(
            SwerveDriveSubsystem drivetrain,
            SelfTestListener listener,
            boolean direct,
            boolean rotation) {
        m_drivetrain = drivetrain;
        m_listener = listener;
        m_timer = new Timer();
        m_direct = direct;
        m_rotation = rotation;
    }

    @Override
    public String getName() {
        return super.getName() + String.format(" direct: %b rotation: %b", m_direct, m_rotation);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_initial = m_drivetrain.getPose();
        prevOscillateTheta = Experiments.instance.enabled(Experiment.OscillateTheta);
        prevOscillateDirect = Experiments.instance.enabled(Experiment.OscillateDirect);
        Experiments.instance.testOverride(Experiment.OscillateTheta, m_rotation);
        Experiments.instance.testOverride(Experiment.OscillateDirect, m_direct);
        terminate = false;
    }

    @Override
    public void execute() {
        if (GeometryUtil.distance(m_drivetrain.getPose(), m_initial) > kMaxDistance) {
            m_listener.fail(this, "Too far from initial pose");
            terminate = true;
        }
    }

    @Override
    public boolean isFinished() {
        return terminate || m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        // put it back the way it was
        Experiments.instance.testOverride(Experiment.OscillateTheta, prevOscillateTheta);
        Experiments.instance.testOverride(Experiment.OscillateDirect, prevOscillateDirect);
        m_listener.pass(this, "There are no assertions.");
    }

}

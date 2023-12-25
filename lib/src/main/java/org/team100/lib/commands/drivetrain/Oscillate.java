package org.team100.lib.commands.drivetrain;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.ParabolicWave;
import org.team100.lib.util.SquareWave;
import org.team100.lib.util.TriangleWave;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive back and forth one meter in x.
 * 
 * This command is intended for calibration of the lower levels of the
 * drivetrain control stack.
 * 
 * It uses minimum-time profiles, so the acceleration is a square wave (i.e.
 * infinite jerk), velocity is a triangle wave, so the resulting position should
 * be a piecewise parabolic curve that looks a lot like a sine wave, though it
 * is not one.
 * 
 * The output can be used to drive the {@link SwerveModuleState} directly, or at
 * the {@link ChassisSpeeds} level, using {@link Experiment.OscillateDirect}.
 */
public class Oscillate extends Command {
    private static final double kAccel = 1;
    private static final double kMaxSpeed = 1;

    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final SwerveDriveSubsystem m_swerve;
    private final SquareWave m_square;
    private final TriangleWave m_triangle;
    private final ParabolicWave m_parabola;
    private final Timer m_timer;
    private SwerveState m_initial;

    public Oscillate(Experiments experiments, SwerveDriveSubsystem swerve) {
        m_experiments = experiments;
        m_swerve = swerve;
        double period = 4 * kMaxSpeed / kAccel;
        m_square = new SquareWave(kAccel, period);
        m_triangle = new TriangleWave(kMaxSpeed, period);
        m_parabola = new ParabolicWave(kMaxSpeed * period / 4, period);
        m_timer = new Timer();
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_initial = m_swerve.getState();
    }

    @Override
    public void execute() {
        double time = m_timer.get();

        double accelM_S_S = m_square.applyAsDouble(time);
        double speedM_S = m_triangle.applyAsDouble(time);
        double positionM = m_parabola.applyAsDouble(time);

        if (m_experiments.enabled(Experiment.OscillateDirect)) {
            // there are four states here because state is mutable :-(
            SwerveModuleState[] states = new SwerveModuleState[] {
                    new SwerveModuleState(speedM_S, GeometryUtil.kRotationZero),
                    new SwerveModuleState(speedM_S, GeometryUtil.kRotationZero),
                    new SwerveModuleState(speedM_S, GeometryUtil.kRotationZero),
                    new SwerveModuleState(speedM_S, GeometryUtil.kRotationZero)
            };
            m_swerve.setRawModuleStates(states);
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speedM_S, 0, 0);
            m_swerve.setChassisSpeeds(chassisSpeeds);
        }

        t.log(Level.DEBUG, "/oscillate/time", time);
        t.log(Level.DEBUG, "/oscillate/setpoint/accel", accelM_S_S);
        t.log(Level.DEBUG, "/oscillate/setpoint/speed", speedM_S);
        t.log(Level.DEBUG, "/oscillate/setpoint/position", positionM);

        SwerveState swerveState = m_swerve.getState();
        // TODO: the acceleration from swerve.getState() is wrong.
        t.log(Level.DEBUG, "/oscillate/measurement/accel", "fixme" /*swerveState.x().a()*/);
        t.log(Level.DEBUG, "/oscillate/measurement/speed", swerveState.x().v());
        t.log(Level.DEBUG, "/oscillate/measurement/position",
                swerveState.x().x() - m_initial.x().x());
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}

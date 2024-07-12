package org.team100.lib.commands.drivetrain;

import java.util.Optional;

import org.team100.lib.commands.Command100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.ParabolicWave;
import org.team100.lib.util.SquareWave;
import org.team100.lib.util.TriangleWave;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Drive back and forth forever.
 * 
 * Drive x or theta via {@link Experiment.OscillateTheta}.
 * Drive chassis speed or module state via {@link Experiment.OscillateDirect}.
 * 
 * This command is intended for calibration of the lower levels of the
 * drivetrain control stack.
 * 
 * It uses minimum-time profiles, so the acceleration is a square wave (i.e.
 * infinite jerk), velocity is a triangle wave, so the resulting position should
 * be a piecewise parabolic curve that looks a lot like a sine wave, though it
 * is not one.
 */
public class Oscillate extends Command100 {
    private static final double kAccel = 1;
    private static final double kMaxSpeed = 1;

    private final SwerveDriveSubsystem m_swerve;
    private final SquareWave m_square;
    private final TriangleWave m_triangle;
    private final ParabolicWave m_parabola;
    private final Timer m_timer;
    private final double m_period;

    public double getPeriod() {
        return m_period;
    }

    private SwerveState m_initial;

    public Oscillate(Logger parent, SwerveDriveSubsystem swerve) {
        super(parent);
        m_swerve = swerve;
        m_period = 4 * kMaxSpeed / kAccel;
        m_square = new SquareWave(kAccel, m_period);
        m_triangle = new TriangleWave(kMaxSpeed, m_period);
        m_parabola = new ParabolicWave(kMaxSpeed * m_period / 4, m_period);
        m_timer = new Timer();
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        m_timer.restart();
        m_initial = m_swerve.getState();
    }

    @Override
    public void execute100(double dt) {
        double time = m_timer.get();

        double accelM_S_S = m_square.applyAsDouble(time);
        double speedM_S = m_triangle.applyAsDouble(time);
        double positionM = m_parabola.applyAsDouble(time);

        if (Experiments.instance.enabled(Experiment.OscillateDirect)) {
            if (Experiments.instance.enabled(Experiment.OscillateTheta)) {
                SwerveModuleState100[] states = new SwerveModuleState100[] {
                        new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(3 * Math.PI / 4))),
                        new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(Math.PI / 4))),
                        new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                        new SwerveModuleState100(speedM_S, Optional.of(new Rotation2d(-1 * Math.PI / 4)))
                };
                m_swerve.setRawModuleStates(states);

            } else {
                SwerveModuleState100[] states = new SwerveModuleState100[] {
                        new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)),
                        new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)),
                        new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero)),
                        new SwerveModuleState100(speedM_S, Optional.of(GeometryUtil.kRotationZero))
                };
                m_swerve.setRawModuleStates(states);

            }
        } else {
            if (Experiments.instance.enabled(Experiment.OscillateTheta)) {
                double speedRad_S = speedM_S;
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, speedRad_S);
                m_swerve.setChassisSpeeds(chassisSpeeds, dt);
            } else {
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speedM_S, 0, 0);
                m_swerve.setChassisSpeeds(chassisSpeeds, dt);
            }

        }

        m_logger.logDouble(Level.TRACE, "period", () -> m_period);
        m_logger.logDouble(Level.TRACE, "time", () -> time);
        m_logger.logDouble(Level.TRACE, "setpoint/accel", () -> accelM_S_S);
        m_logger.logDouble(Level.TRACE, "setpoint/speed", () -> speedM_S);
        m_logger.logDouble(Level.TRACE, "setpoint/position", () -> positionM);

        SwerveState swerveState = m_swerve.getState();
        if (Experiments.instance.enabled(Experiment.OscillateTheta)) {
            m_logger.logDouble(Level.TRACE, "measurement/speed", () -> swerveState.theta().v());
            m_logger.logDouble(Level.TRACE, "measurement/position",
                    () -> swerveState.theta().x() - m_initial.theta().x());
        } else {
            m_logger.logDouble(Level.TRACE, "measurement/speed", () -> swerveState.x().v());
            m_logger.logDouble(Level.TRACE, "measurement/position",
                    () -> swerveState.x().x() - m_initial.x().x());
        }
    }

    @Override
    public void end100(boolean interrupted) {
        m_swerve.stop();
    }

}

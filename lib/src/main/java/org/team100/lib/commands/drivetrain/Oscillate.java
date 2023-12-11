package org.team100.lib.commands.drivetrain;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.ParabolicWave;
import org.team100.lib.util.SquareWave;
import org.team100.lib.util.TriangleWave;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command is intended for calibration of the lower levels of the
 * drivetrain control stack.
 * 
 * It drives back and forth in x, using minimum-time profiles, so the
 * acceleration is a square wave, velocity is a triangle wave, delivered
 * directly to the drive velocity servos, and the resulting position should be a
 * piecewise parabolic curve.
 */
public class Oscillate extends Command {
    private static final double kAccel = 1;
    private static final double kMaxSpeed = 1;
    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystemInterface m_swerve;
    private final SquareWave m_square;
    private final TriangleWave m_triangle;
    private final ParabolicWave m_parabola;
    private final Timer m_timer;
    private SwerveState m_initial;

    public Oscillate(SwerveDriveSubsystemInterface swerve) {
        m_swerve = swerve;
        double period = 4 * kMaxSpeed / kAccel;
        m_square = new SquareWave(kAccel, period);
        m_triangle = new TriangleWave(kMaxSpeed, period);
        m_parabola = new ParabolicWave(kMaxSpeed * period / 4, period);
        m_timer = new Timer();
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_initial = m_swerve.getState();
    }

    @Override
    public void execute() {
        double time = m_timer.get();

        double accel = m_square.applyAsDouble(time);
        double speed = m_triangle.applyAsDouble(time);
        double position = m_parabola.applyAsDouble(time);

        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(speed, GeometryUtil.kRotationZero),
                new SwerveModuleState(speed, GeometryUtil.kRotationZero),
                new SwerveModuleState(speed, GeometryUtil.kRotationZero),
                new SwerveModuleState(speed, GeometryUtil.kRotationZero)
        };
        m_swerve.setRawModuleStates(states);

        SwerveState swerveState = m_swerve.getState();

        t.log(Level.DEBUG, "/oscillate/time", time);
        t.log(Level.DEBUG, "/oscillate/setpoint/accel", accel);
        t.log(Level.DEBUG, "/oscillate/setpoint/speed", speed);
        t.log(Level.DEBUG, "/oscillate/setpoint/position", position);

        // TODO: the acceleration from swerve.getState() is wrong.
        t.log(Level.DEBUG, "/oscillate/measurement/accel", swerveState.x().a());
        t.log(Level.DEBUG, "/oscillate/measurement/speed", swerveState.x().v());
        t.log(Level.DEBUG, "/oscillate/measurement/position",
                swerveState.x().x() - m_initial.x().x());
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

}

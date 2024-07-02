package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.SquareWave;

import edu.wpi.first.wpilibj.Timer;

/**
 * Drive back and forth, fast, while spinning, to explore the causes of veering.
 */
public class Veering extends Command100 {
    /** translation command in m/s */
    private static final double kAmplitude = 2;
    private static final double kPeriod = 10;
    /** omega command in rad/s */
    private static final double kOmega = Math.PI;
    private final SwerveDriveSubsystem m_swerve;
    private final SquareWave m_square;
    private final Timer m_timer;

    public Veering(SwerveDriveSubsystem swerve) {
        m_swerve = swerve;
        m_square = new SquareWave(kAmplitude, kPeriod);
        m_timer = new Timer();
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        m_timer.restart();
    }

    @Override
    public void execute100(double dt) {
        double time = m_timer.get();
        double dx = m_square.applyAsDouble(time);
        FieldRelativeVelocity input = new FieldRelativeVelocity(dx, 0, kOmega);
        m_swerve.driveInFieldCoords(input, dt);
        t.log(Level.TRACE, "input", input);
    }

    @Override
    public void end100(boolean interrupted) {
        m_swerve.stop();
    }
}

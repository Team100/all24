package org.team100.frc2024.commands;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootPreload extends Command implements Glassy {
    private final Intake m_intake;
    private final SensorInterface m_sensor;
    private final FeederSubsystem m_feeder;
    private final DrumShooter m_shooter;
    private final Timer m_timer;
    private final SwerveDriveSubsystem m_drive;
    private final boolean m_isPreload;

    private final BooleanLogger m_log_at_speed;
    private final DoubleLogger m_log_timer;
    private final DoubleLogger m_log_pivot_error;

    private boolean atVelocity = false;
    private boolean finished = false;

    public ShootPreload(
            LoggerFactory parent,
            SensorInterface sensor,
            DrumShooter shooter,
            Intake intake,
            FeederSubsystem feeder,
            SwerveDriveSubsystem drive,
            boolean isPreload) {
        LoggerFactory child = parent.child(this);
        m_log_at_speed = child.booleanLogger(Level.TRACE, "at speed");
        m_log_timer = child.doubleLogger(Level.TRACE, "timer");
        m_log_pivot_error = child.doubleLogger(Level.TRACE, "pivot error");
        m_sensor = sensor;
        m_shooter = shooter;
        m_intake = intake;
        m_feeder = feeder;
        m_drive = drive;
        m_isPreload = isPreload;
        m_timer = new Timer();
        addRequirements(m_intake, m_feeder, m_shooter);
    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;
        m_timer.reset();
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;

        Translation2d robotLocation = m_drive.getPose().getTranslation();
        Translation2d speakerLocation = ShooterUtil.getSpeakerTranslation(alliance.get());

        double rangeM = robotLocation.getDistance(speakerLocation);
        double pivotSetpointRad = ShooterUtil.getAngleRad(rangeM);

        m_shooter.setAngle(pivotSetpointRad);
        m_shooter.forward();

        if (!m_sensor.getFeederSensor() && !m_sensor.getIntakeSensor()) {
            m_intake.stop();
            m_feeder.stop();
            OptionalDouble shooterPivotPosition = m_shooter.getPivotPosition();
            if (shooterPivotPosition.isPresent()) {
                double error = shooterPivotPosition.getAsDouble() - pivotSetpointRad;
                m_log_pivot_error.log(() -> error);
                if (m_shooter.atVelocitySetpoint(m_isPreload)
                        && Math.abs(error) < 0.01) {
                    atVelocity = true;
                    m_timer.start();
                }
            }
        }

        if (atVelocity) {
            m_feeder.feed();
            m_intake.intake();

            if (m_timer.get() > 0.2) {
                finished = true;
            }
        }

        m_log_at_speed.log(() -> atVelocity);
        m_log_timer.log(m_timer::get);
    }

    @Override
    public void end(boolean interrupted) {
        atVelocity = false;
        finished = false;
        m_timer.stop();
        m_shooter.stop();
        m_intake.stop();
        m_feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}

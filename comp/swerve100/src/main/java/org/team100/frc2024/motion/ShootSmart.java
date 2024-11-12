package org.team100.frc2024.motion;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSmart extends Command implements Glassy {
    private final Intake m_intake;
    private final SensorInterface m_sensor;
    private final FeederSubsystem m_feeder;
    private final DrumShooter m_shooter;
    private final SwerveDriveSubsystem m_drive;
    private final boolean m_isPreload;

    private boolean atVelocity;

    public ShootSmart(
            SensorInterface sensor,
            DrumShooter shooter,
            Intake intake,
            FeederSubsystem feeder,
            SwerveDriveSubsystem drive,
            boolean isPreload) {
        m_intake = intake;
        m_sensor = sensor;
        m_feeder = feeder;
        m_shooter = shooter;
        m_drive = drive;
        m_isPreload = isPreload;
        addRequirements(m_intake, m_feeder, m_shooter);
    }

    @Override
    public void initialize() {
        atVelocity = false;
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

        // no matter the note position, set the shooter angle and speed
        m_shooter.setAngle(pivotSetpointRad);
        m_shooter.forward();

        m_intake.runLowerIntake();

        if (m_sensor.getFeederSensor() || m_sensor.getIntakeSensor()) {
            // if either sensor sees light, note is not in position yet, so feed
            m_intake.intake();
            m_feeder.feed();
        } else {
            // both sensors are dark, note is in position, wait for drums to spin up
            m_intake.stop();
            m_feeder.stop();
            OptionalDouble shooterPivotPosition = m_shooter.getPivotPosition();
            if (shooterPivotPosition.isPresent()) {
                double errorRad = shooterPivotPosition.getAsDouble() - pivotSetpointRad;
                if (m_shooter.atVelocitySetpoint(m_isPreload) && Math.abs(errorRad) < 0.01) {
                    // latch ready
                    atVelocity = true;
                }
            }
        }
        if (atVelocity) {
            // if ready, shoot
            m_feeder.feed();
            m_intake.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_intake.stop();
        m_feeder.stop();
    }

}

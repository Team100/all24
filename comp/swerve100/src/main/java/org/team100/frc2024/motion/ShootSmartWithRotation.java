package org.team100.frc2024.motion;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.frc2024.motion.drivetrain.manual.ManualWithShooterLock;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.commands.Command100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShootSmartWithRotation extends Command100{
    private final Shooter m_shooter; 
    private static final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystem m_drive; 
    private final ManualWithShooterLock m_driver;
    private final FeederSubsystem m_feeder;
    private final Intake m_intake;
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    public ShootSmartWithRotation(SwerveDriveSubsystem drive, Shooter shooter, FeederSubsystem feeder, Intake intake,
    ManualWithShooterLock driver,
    Supplier<DriverControl.Velocity> twistSupplier) {
        m_shooter = shooter;
        m_drive = drive;
        m_intake = intake;
        m_driver = driver;
        m_feeder = feeder;
        m_twistSupplier = twistSupplier;
        addRequirements(m_intake, m_feeder, m_shooter, m_drive);
    }
    
    @Override
    public void initialize100() {
        m_driver.reset(m_drive.getPose());
    }

    public void execute100(double dt) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return;
        }
        m_shooter.forward();
        Translation2d robotLocation = m_drive.getPose().getTranslation();
        Translation2d speakerLocation = ShooterUtil.getSpeakerTranslation(alliance.get());
        Translation2d difference = robotLocation.minus(speakerLocation);
        double angle = MathUtil.angleModulus(Math.atan2(difference.getY(),difference.getX())-Math.PI);
        t.log(Level.DEBUG, m_name, "angle", angle);
        double angleModulus = MathUtil.angleModulus(m_drive.getPose().getRotation().getRadians());
        t.log(Level.DEBUG, m_name, "realangle", angle);
        double angleError = angle-angleModulus;
        double distance = robotLocation.getDistance(speakerLocation);
        m_shooter.setAngle(ShooterUtil.getAngleRad(distance));
        double rangeM = robotLocation.getDistance(speakerLocation);
        double angleRad = ShooterUtil.getAngleRad(rangeM);
        double errorRad = m_shooter.getPivotPosition() - angleRad;
        FieldRelativeVelocity twist = m_driver.apply(m_drive.getState(), m_twistSupplier.get());
        m_drive.driveInFieldCoords(twist, 0.02);
        if (Math.hypot(m_drive.getState().y().v(), m_drive.getState().x().v())>0.01) {
            return;
        }
        if (m_shooter.atVelocitySetpoint() && Math.abs(errorRad) < 0.01 && Math.abs(angleError) <0.05) {
            m_feeder.feed();
            m_intake.intake();
        }
    }
}

package org.team100.frc2024;

import java.util.Optional;

import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.VisionDataProvider24;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    /**
     * flash if the vision input is older than 0.3 sec. Typical "good" frame rate is
     * 15 hz, so 0.06 sec, much less.
     */
    private static final long kPersistenceUs = 300000;

    private final LEDIndicator m_indicator;
    private final SensorInterface m_sensors;
    private final Shooter m_shooter;
    private final VisionDataProvider24 m_vision;

    /**
     * 
     * @param indicator output
     * @param sensors   green when note in position
     * @param shooter   purple when at shooting speed
     * @param vision    flash when vision has a poor fix, solid when it's good.
     * 
     */
    public LEDSubsystem(
            LEDIndicator indicator,
            SensorInterface sensors,
            Shooter shooter,
            VisionDataProvider24 vision) {
        m_indicator = indicator;
        m_sensors = sensors;
        m_shooter = shooter;
        m_vision = vision;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isDSAttached() || DriverStation.isDisabled()) {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    m_indicator.setStripSolid(0, State.RED);
                } else {
                    m_indicator.setStripSolid(0, State.BLUE);
                }
            } else {
                m_indicator.setStripSolid(0, State.ORANGE);
            }
        } else {
            boolean atVelocitySetpoint = m_shooter.atVelocitySetpoint(false);
            SmartDashboard.putBoolean("VELOCITY", atVelocitySetpoint);
            if (atVelocitySetpoint) {
                m_indicator.setStripSolid(0, State.PURPLE);
            } else {
                boolean indexerIsEmpty = m_sensors.getFeederSensor();
                SmartDashboard.putBoolean("FEEDER", indexerIsEmpty);
                if (indexerIsEmpty) {
                    m_indicator.setStripSolid(0, State.RED);
                } else {
                    m_indicator.setStripSolid(0, State.GREEN);
                }
            }
        }

        long poseAgeUs = m_vision.getPoseAgeUs();

        // flash if the pose is too old
        m_indicator.setFlashing(poseAgeUs > kPersistenceUs);

        m_indicator.periodic();
    }
}

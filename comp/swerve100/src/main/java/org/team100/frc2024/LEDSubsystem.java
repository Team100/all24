package org.team100.frc2024;

import java.util.Optional;

import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    LEDIndicator m_indicator;
    SensorInterface m_sensors;
    Shooter m_shooter;

    public LEDSubsystem(LEDIndicator indicator, SensorInterface sensors, Shooter shooter) {
        m_indicator = indicator;
        m_sensors = sensors;
        m_shooter = shooter;
    }

    @Override
    public void periodic() {
        boolean atVelocitySetpoint = m_shooter.atVelocitySetpoint(false);
        boolean indexerIsEmpty = m_sensors.getFeederSensor();
        SmartDashboard.putBoolean("FEEDER", indexerIsEmpty);
        SmartDashboard.putBoolean("VELOCITY", atVelocitySetpoint);

        if (!DriverStation.isDSAttached() || DriverStation.isDisabled()) {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    m_indicator.setStripSolid(0, State.RED);
                } else {
                    m_indicator.setStripSolid(0, State.BLUE);
                }
            } else {
                m_indicator.setStripFlashing(0, State.ORANGE, 80000);
            }
        } else {
            if (atVelocitySetpoint) {
                m_indicator.setStripSolid(0, State.PURPLE);
            } else {
                if (indexerIsEmpty) {
                    m_indicator.setStripSolid(0, State.RED);
                } else {
                    m_indicator.setStripSolid(0, State.GREEN);
                }
            }
        }

        m_indicator.periodic();
    }
}

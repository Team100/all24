package org.team100.frc2024;

import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;

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

        if(m_shooter.atVelocitySetpoint(false)){
            m_indicator.setStripSolid(0, State.PURPLE);
        } else {
            if (m_sensors.getFeederSensor()) {
                m_indicator.setStripSolid(0, State.RED);
            } else {
                m_indicator.setStripSolid(0, State.GREEN);
            }
        }

        SmartDashboard.putBoolean("FEEDER", m_sensors.getFeederSensor());
        SmartDashboard.putBoolean("VELOCITY", m_shooter.atVelocitySetpoint(false));


        m_indicator.periodic();
        // m_indicator.setStripGreen(0, State.RED);
    }
}


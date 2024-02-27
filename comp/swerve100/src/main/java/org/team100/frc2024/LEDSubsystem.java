package org.team100.frc2024;

import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    LEDIndicator m_indicator;
    SensorInterface m_sensors;

    public LEDSubsystem(LEDIndicator indicator, SensorInterface sensors) {
        m_indicator = indicator;
        m_sensors = sensors;
    }

    @Override
    public void periodic() {
        if (m_sensors.getFeederSensor()) {
            m_indicator.setStripSolid(0, State.RED);
        } else {
            m_indicator.setStripSolid(0, State.GREEN);

        }

        m_indicator.periodic();
        // m_indicator.setStripGreen(0, State.RED);
    }
}

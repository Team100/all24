package org.team100.lib.indicator;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIndicator {
    public static class Config {
        public int kStripLength = 60;
    }

    private final Config m_config = new Config();

    /**
     * This enum exists to prepopulate the buffers, so they can be set atomically,
     * which is faster.
     * TODO: fix these colors, i think they're all wrong now
     */
    public static enum State {
        BLACK(new Color(0, 0, 0)),
        RED(new Color(255, 0, 0)),
        GREEN(new Color(0, 0, 255)),
        PURPLE(new Color(255, 100, 0)),
        YELLOW(new Color(255, 0, 80)),
        ORANGE(new Color(255, 0, 30));

        private final Color color;

        private State(Color color) {
            this.color = color;
        }
    }

    private final Map<State, AddressableLEDBuffer> buffers;
    private final AddressableLED led;

    public LEDIndicator(int port) {
        buffers = new HashMap<State, AddressableLEDBuffer>();
        for (State s : State.values()) {
            AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_config.kStripLength);
            for (int i = 0; i < m_config.kStripLength; i++) {
                buffer.setLED(i, s.color);
            }
            buffers.put(s, buffer);
        }
        led = new AddressableLED(port);
        led.setLength(m_config.kStripLength);
        led.start();
        set(State.ORANGE);
    }

    public void set(State s) {
        led.setData(buffers.get(s));
    }

    public void close() {
        led.close();
    }
}

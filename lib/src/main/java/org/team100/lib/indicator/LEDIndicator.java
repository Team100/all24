package org.team100.lib.indicator;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * An LED strip used as a signal light.
 * 
 * Use sthe AddressableLED feature of the RoboRIO.
 */
public class LEDIndicator {
    private static final int kStripLength = 256;
    private static final double brightnessScaler = 50.0/255.0;
    private State currentColor;
    
    /**
     * This enum exists to prepopulate the buffers, so they can be set atomically,
     * which is faster.
     */
    public enum State {
        BLACK(new Color((int)(0*brightnessScaler), (int)(0*brightnessScaler), (int)(0*brightnessScaler))),
        RED(new Color((int)(255*brightnessScaler), (int)(0*brightnessScaler), (int)(0*brightnessScaler))),
        GREEN(new Color((int)(0*brightnessScaler), (int)(255*brightnessScaler), (int)(0*brightnessScaler))),
        BLUE(new Color((int)(0*brightnessScaler),(int)(0*brightnessScaler),(int)(255*brightnessScaler))),
        PURPLE(new Color((int)(255*brightnessScaler), (int)(0*brightnessScaler), (int)(255*brightnessScaler))),
        YELLOW(new Color((int)(255*brightnessScaler), (int)(255*brightnessScaler), (int)(0*brightnessScaler))),
        WHITE(new Color ((int)(255*brightnessScaler),(int)(255*brightnessScaler),(int)(255*brightnessScaler))),
        ORANGE(new Color((int)(255*brightnessScaler), (int)(80*brightnessScaler), (int)(0*brightnessScaler)));

        private final Color color;

        private State(Color color) {
            this.color = color;
        }
    }

    private final Map<State, AddressableLEDBuffer> buffers;
    private final AddressableLED led;

    public LEDIndicator(int port) {
        buffers = new EnumMap<>(State.class);
        for (State s : State.values()) {
            AddressableLEDBuffer buffer = new AddressableLEDBuffer(kStripLength);
            for (int i = 0; i < kStripLength; i++) {
                buffer.setLED(i, s.color);
            }
            buffers.put(s, buffer);
        }
        led = new AddressableLED(port);
        led.setLength(kStripLength);
        led.start();
        set(State.BLACK);
    }

    public void set(State s) {
        led.setData(buffers.get(s));
    }

    public void close() {
        led.close();
    }

    public void displayTeam100() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(kStripLength);
        for (int i = 0; i < kStripLength; i++) {
            if( Arrays.asList(165,153,154,155,156,157,158,138,139,140,141,129,134,122,123,124,125,106,107,108,109,97,102,90,91,92,93).contains(i) ){
                currentColor = State.WHITE;
            }
            else {
                currentColor = State.ORANGE;
            }
            buffer.setLED(i, currentColor.color);
        }
        led.setData(buffer);
    }
}

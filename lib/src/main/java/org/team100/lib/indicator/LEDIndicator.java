package org.team100.lib.indicator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

/**
 * An LED strip used as a signal light.
 * 
 * Uses the AddressableLED feature of the RoboRIO.
 * 
 * We use these strips: https://www.amazon.com/gp/product/B01CNL6LLA
 * 
 * Note these strips use a different order: red-blue-green, not
 * red-green-blue, so the colors need some fixing up.
 */
public class LEDIndicator {
    /**
     * Maps indicator colors to WS2811 colors.
     */
    public enum State {
        BLACK(Color.kBlack),
        RED(Color.kRed),
        BLUE(Color.kBlue),
        GREEN(Color.kLime),
        PURPLE(Color.kFuchsia),
        YELLOW(Color.kYellow),
        ORANGE(Color.kOrange),
        WHITE(Color.kWhite);

        /**
         * This "color" is what we tell the LED strip to make it display the actual
         * desired color.
         */
        private final Color color;

        /**
         * @param color the correct RGB color
         */
        private State(Color color) {
            if (RobotBase.isSimulation()) {
                // use RGB colors
                this.color = color;
            } else {
                // swap blue and green to make RBG
                this.color = new Color(color.red, color.blue, color.green);
            }
        }
    }

    /**
     * Fast flashing, 15hz.
     */
    private static final int kFlashDurationMicrosec = 30000;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBuffer blackBuffer;
    private final List<LEDStrip> leds = new ArrayList<>();
    private boolean flashing;
    private final int m_numstrips;
    private final int m_numLEDs;

    public LEDIndicator(int port, int numLEDs, LEDStrip... strips) {
        int numstrips = 0;
        m_numLEDs = numLEDs;
        for (LEDStrip strip : strips) {
            numstrips++;
        }
        m_numstrips = numstrips;
        Collections.addAll(leds, strips);
        int length = leds.stream().map(LEDStrip::getLength).reduce(0, Integer::sum);
        led = new AddressableLED(port);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        blackBuffer = new AddressableLEDBuffer(length);
        // for (int i = 0; i < m_numstrips; i++) {
        //     Patterns.solid(leds.get(i * m_numLEDs + 1), blackBuffer, Color.kWhite);
        // }
        led.setData(buffer);
        led.start();
    }

    public void setStripSolid(LEDStrip.strip state, State s) {
        if (state.equals(LEDStrip.strip.BACKLIGHT)) {
            for (int i = 0; i < m_numstrips; i++) {
                Patterns.solid(leds.get(i * m_numLEDs + 1), buffer, s.color);
                Patterns.solid(leds.get(i * m_numLEDs + 1), blackBuffer, s.color);
            }
        } else {
            for (int i = 0; i < m_numstrips; i++) {
                Patterns.solid(leds.get(i * m_numLEDs), buffer, s.color);
            }
        }
    }

    public void setFlashing(boolean flashing) {
        this.flashing = flashing;
    }

    public void periodic() {
        if (flashing) {
            if ((RobotController.getFPGATime() / kFlashDurationMicrosec) % 2 == 0) {
                led.setData(buffer);
            } else {
                led.setData(blackBuffer);
            }
        } else {
            led.setData(buffer);
        }
    }

    /////////////////////////////////
    // work in progress

    void setStripRainbow(LEDStrip strip) {
        Patterns.rainbow(strip, buffer);
    }

    void setStripChase(LEDStrip strip) {
        Color[] colors = { new Color(), new Color() };
        Patterns.chase(colors, strip, buffer);
    }
}

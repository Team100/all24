package org.team100.lib.indicator;

import java.util.ArrayList;
import java.util.Arrays;
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
 * 
 * This copy includes stuff about the 5-volt LED panel, which
 * we decided not to use in 2024.
 */
public class LEDIndicator2 {
    // keeps the current below uh 2 amps i think?
    private static final double brightnessScaler = 50.0 / 255.0;

    /**
     * Maps indicator colors to WS2811 colors.
     */
    public enum State {
        BLACK(new Color(0, 0, 0)),
        RED(new Color(brightnessScaler, 0, 0)),
        GREEN(new Color(0, brightnessScaler, 0)),
        BLUE(new Color(0, 0, brightnessScaler)),
        PURPLE(new Color(brightnessScaler, 0, brightnessScaler)),
        YELLOW(new Color(brightnessScaler, brightnessScaler, 0)),
        WHITE(new Color(brightnessScaler, brightnessScaler, brightnessScaler)),
        ORANGE(new Color(brightnessScaler, 0.31 * brightnessScaler, 0));

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
    private final List<LEDStrip> strips;

    private State m_front;
    private State m_back;
    private boolean m_flashing;
    private static final int kStripLength = 256;
    private State currentColor;

    public LEDIndicator2(int port) {
        strips = new ArrayList<>();

        strips.add(new LEDStrip(0, 16));
        strips.add(new LEDStrip(16, 32));
        strips.add(new LEDStrip(32, 48));
        strips.add(new LEDStrip(48, 64));
        strips.add(new LEDStrip(64, 80));
        strips.add(new LEDStrip(80, 96));
        strips.add(new LEDStrip(96, 112));
        strips.add(new LEDStrip(112, 128));
        strips.add(new LEDStrip(128, 144));
        strips.add(new LEDStrip(144, 160));

        int length = strips.stream().map(LEDStrip::end).reduce(0, Integer::max);

        led = new AddressableLED(port);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        led.setData(buffer);
        led.start();
        m_flashing = false;
    }

    public void setGroup(LEDGroup group, Color c) {
        for (LEDStrip strip : strips) {
            if (strip.group() == group) {
                strip.setColor(c);
            }
        }
    }

    public void setFlashing(boolean flashing) {
        m_flashing = flashing;
    }

    /**
     * Periodic does all the real work in this class.
     */
    public void periodic() {
        // back always shows the same
        for (LEDStrip strip : strips) {
            strip.solid(buffer, m_back.color);
        }

        // front depends on flashing state
        if (m_flashing) {
            if ((RobotController.getFPGATime() / kFlashDurationMicrosec) % 2 == 0) {
                for (LEDStrip strip : strips) {
                    strip.solid(buffer, Color.kBlack);
                }
            } else {
                for (LEDStrip strip : strips) {
                    strip.solid(buffer, m_front.color);
                }
            }
        } else {
            for (LEDStrip strip : strips) {
                strip.solid(buffer, m_front.color);
            }
        }

        // update the output with the buffer we constructed.
        led.setData(buffer);
    }

    public void displayTeam100() {
        AddressableLEDBuffer buffer2 = new AddressableLEDBuffer(kStripLength);
        for (int i = 0; i < kStripLength; i++) {
            if (Arrays.asList(165, 153, 154, 155, 156, 157, 158, 138, 139, 140, 141, 129, 134, 122, 123, 124, 125, 106,
                    107, 108, 109, 97, 102, 90, 91, 92, 93).contains(i)) {
                currentColor = State.WHITE;
            } else {
                currentColor = State.ORANGE;
            }
            buffer2.setLED(i, currentColor.color);
        }
        led.setData(buffer2);
    }

    public void setStripRainbow(LEDStrip strip) {
        Patterns.rainbow(strip, buffer);

    }

    public void setStripChase(LEDStrip strip) {
        Color[] colors = { new Color(), new Color() };
        Patterns.chase(colors, strip, buffer);
    }
}

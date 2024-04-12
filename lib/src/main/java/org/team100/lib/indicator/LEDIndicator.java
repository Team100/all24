package org.team100.lib.indicator;

import java.util.ArrayList;
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

    /** Turns flashing on or off, in case you find it annoying. */
    private static final boolean kFlash = false;

    /** Maps indicator colors to WS2811 colors. */
    public enum State {

        BLACK(Color.kBlack),
        RED(Color.kRed),
        BLUE(Color.kBlue),
        GREEN(Color.kLime),
        PURPLE(Color.kFuchsia),
        YELLOW(Color.kYellow),
        ORANGE(Color.kOrange),
        WHITE(Color.kBlack); // turn off white to save battery

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
    private final List<LEDStrip> m_frontStrips;
    private final List<LEDStrip> m_backStrips;

    private State m_front;
    private State m_back;
    private boolean m_flashing;

    public LEDIndicator(int port) {
        m_frontStrips = new ArrayList<>();
        m_backStrips = new ArrayList<>();

        m_frontStrips.add(new LEDStrip(0, 8));
        m_backStrips.add(new LEDStrip(8, 18));
        m_frontStrips.add(new LEDStrip(18, 27));
        m_backStrips.add(new LEDStrip(27, 37));
        m_frontStrips.add(new LEDStrip(37, 46));
        m_backStrips.add(new LEDStrip(46, 55));

        int length = Math.max(
                m_frontStrips.stream().map(LEDStrip::end).reduce(0, Integer::max),
                m_backStrips.stream().map(LEDStrip::end).reduce(0, Integer::max));
        led = new AddressableLED(port);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        led.setData(buffer);
        led.start();

        m_flashing = false;

    }

    public void setFront(State s) {
        m_front = s;
    }

    public void setBack(State s) {
        m_back = s;
    }

    public void setFlashing(boolean flashing) {
        m_flashing = flashing;
    }

    /**
     * Periodic does all the real work in this class.
     */
    public void periodic() {
        // back always shows the same
        for (LEDStrip strip : m_backStrips) {
            strip.solid(buffer, m_back.color);
        }

        if (kFlash) {
            // front depends on flashing state
            if (m_flashing) {
                if ((RobotController.getFPGATime() / kFlashDurationMicrosec) % 2 == 0) {
                    for (LEDStrip strip : m_frontStrips) {
                        strip.solid(buffer, Color.kBlack);
                    }
                } else {
                    for (LEDStrip strip : m_frontStrips) {
                        strip.solid(buffer, m_front.color);
                    }
                }
            } else {
                for (LEDStrip strip : m_frontStrips) {
                    strip.solid(buffer, m_front.color);
                }
            }
        } else {
            for (LEDStrip strip : m_frontStrips) {
                strip.solid(buffer, m_front.color);
            }
        }

        // update the output with the buffer we constructed.
        led.setData(buffer);
    }
}

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
public class LEDIndicator2 {
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
    private final List<LEDStrip> strips;

    private State m_front;
    private State m_back;
    private boolean m_flashing;

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

    public void setGroup(LEDGroup group, Color c){
        for(LEDStrip strip : strips){
            if(strip.group() == group){
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
}

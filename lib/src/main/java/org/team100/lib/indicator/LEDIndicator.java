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
    private final List<LEDStrip> backleds = new ArrayList<>();
    private final List<LEDStrip> frontleds = new ArrayList<>();
    private State frontState;
    private State backState;
    private boolean backflashing = false;
    private boolean frontflashing = false;
    private final int m_numsplitstrips;

    public LEDIndicator(int port) {
        m_numsplitstrips = 10;
        // TODO get real strips that are front and back
        List<LEDStrip> allLeds = new ArrayList<>();
        for (int i = 0; i < m_numsplitstrips; i++) {
            allLeds.add(new LEDStrip(16, i * 16));
        }
        for (int i = 0; i < m_numsplitstrips / 2; i++) {
            frontleds.add(new LEDStrip(16, i * 2 * 16));
        }
        for (int i = 0; i < m_numsplitstrips / 2; i++) {
            backleds.add(new LEDStrip(16, (i * 2 + 1) * 16));
        }
        int length = allLeds.stream().map(LEDStrip::getLength).reduce(0, Integer::sum);
        led = new AddressableLED(port);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        blackBuffer = new AddressableLEDBuffer(length);
        led.setData(buffer);
        led.start();
    }

    public void setFrontSolid(State s) {
        frontState = s;
        for (int i = 0; i < m_numsplitstrips; i++) {
            Patterns.solid(frontleds.get(i), buffer, frontState.color);
        }
        setFrontFlashing(frontflashing);
    }

    public void setBackSolid(State s) {
        backState = s;
        for (int i = 0; i < m_numsplitstrips; i++) {
            Patterns.solid(backleds.get(i), buffer, backState.color);
        }
        setFrontFlashing(backflashing);
    }

    public void setFrontFlashing(boolean flashing) {
        if (flashing) {
            for (int i = 0; i < m_numsplitstrips / 2; i++) {
                Patterns.solid(frontleds.get(i), blackBuffer, Color.kBlack);
            }
        } else {
            for (int i = 0; i < m_numsplitstrips / 2; i++) {
                Patterns.solid(frontleds.get(i), blackBuffer, frontState.color);
            }
        }
        frontflashing = flashing;
    }

    public void setBackFlashing(boolean flashing) {
        if (flashing) {
            for (int i = 0; i < m_numsplitstrips / 2; i++) {
                Patterns.solid(backleds.get(i), blackBuffer, Color.kBlack);
            }
        } else {
            for (int i = 0; i < m_numsplitstrips / 2; i++) {
                Patterns.solid(backleds.get(i), blackBuffer, backState.color);
            }
        }
        backflashing = flashing;
    }

    public void periodic() {
            if ((RobotController.getFPGATime() / kFlashDurationMicrosec) % 2 == 0) {
                led.setData(buffer);
            } else {
                led.setData(blackBuffer);
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

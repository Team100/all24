package org.team100.lib.indicator;

import java.util.Arrays;
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

    private static final int kStripLength = 256;
    private static final double brightnessScaler = 50.0/255.0;
    private State currentColor;
    

    /**
     * Maps indicator colors to WS2811 colors.
     */
    public enum State {

//         BLACK(new Color((int)(0*brightnessScaler), (int)(0*brightnessScaler), (int)(0*brightnessScaler))),
//         RED(new Color((int)(255*brightnessScaler), (int)(0*brightnessScaler), (int)(0*brightnessScaler))),
//         GREEN(new Color((int)(0*brightnessScaler), (int)(255*brightnessScaler), (int)(0*brightnessScaler))),
//         BLUE(new Color((int)(0*brightnessScaler),(int)(0*brightnessScaler),(int)(255*brightnessScaler))),
//         PURPLE(new Color((int)(255*brightnessScaler), (int)(0*brightnessScaler), (int)(255*brightnessScaler))),
//         YELLOW(new Color((int)(255*brightnessScaler), (int)(255*brightnessScaler), (int)(0*brightnessScaler))),
//         WHITE(new Color ((int)(255*brightnessScaler),(int)(255*brightnessScaler),(int)(255*brightnessScaler))),
//         ORANGE(new Color((int)(255*brightnessScaler), (int)(80*brightnessScaler), (int)(0*brightnessScaler)));


        BLACK(Color.kBlack),
        RED(Color.kRed),
        BLUE(Color.kBlue),
        GREEN(Color.kLime),
        PURPLE(Color.kFuchsia),
        YELLOW(Color.kYellow),
        ORANGE(Color.kOrange),
        WHITE(Color.kBlack);

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
        // TODO: change the ranges to match the actual LEDs
        m_frontStrips = new ArrayList<>();
        m_backStrips = new ArrayList<>();
        
        m_frontStrips.add(new LEDStrip(0, 8));
        m_backStrips.add(new LEDStrip(8, 18));
        m_frontStrips.add(new LEDStrip(18,27));
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

    public void setStripRainbow(LEDStrip strip){
        Patterns.rainbow(strip, buffer);  

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
  
    public void setStripChase(LEDStrip strip){
        Color[] colors = {new Color(), new Color()};
        Patterns.chase(colors, strip, buffer); 
    } 
      
    /**
     * Periodic does all the real work in this class.
     */
    public void periodic() {
        // back always shows the same
        for (LEDStrip strip : m_backStrips) {
            strip.solid(buffer, m_back.color);
        }


        // front depends on flashing state
        // if (m_flashing) {
        //     if ((RobotController.getFPGATime() / kFlashDurationMicrosec) % 2 == 0) {
        //         for (LEDStrip strip : m_frontStrips) {
        //             strip.solid(buffer, Color.kBlack);
        //         }
        //     } else {
        //         for (LEDStrip strip : m_frontStrips) {
        //             strip.solid(buffer, m_front.color);
        //         }
        //     }
        // } else {
            for (LEDStrip strip : m_frontStrips) {
                strip.solid(buffer, m_front.color);
            }
        // }

        // update the output with the buffer we constructed.
        led.setData(buffer);
    }
}

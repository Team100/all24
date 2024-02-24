package org.team100.lib.indicator;

import java.util.ArrayList;
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
    private static final int kStripLength = 60;
    
    
    /**
     * This enum exists to prepopulate the buffers, so they can be set atomically,
     * which is faster.
     */
    public enum State {
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

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private double totalLength = 0;

    ArrayList<LEDStrip> leds = new ArrayList<>();

    public LEDIndicator(int port, LEDStrip strip1, LEDStrip strip2) {
        
        leds.add(strip1);
        leds.add(strip2);

        for(LEDStrip s : leds){
            totalLength += s.getLength();
        }

        led = new AddressableLED(port);
        led.setLength((int)totalLength);
        led.start();


        buffer = new AddressableLEDBuffer((int)totalLength);

        led.setData(buffer);

    }

    public void setStripSolid(LEDStrip strip, State s){
        Patterns.solid(strip, buffer, s.color);
    }

    public void setStripRainbow(LEDStrip strip){
        Patterns.rainbow(strip, buffer);  

    }

    public void setStripChase(LEDStrip strip){
        Color[] colors = {new Color(), new Color()};
        Patterns.chase(colors, strip, buffer);  

    }


    
}

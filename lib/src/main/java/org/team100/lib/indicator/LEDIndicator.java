package org.team100.lib.indicator;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;

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

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private double totalLength = 0;

    ArrayList<LEDStrip> leds = new ArrayList<>();

    public LEDIndicator(int port, LEDStrip... strips) {

        for (LEDStrip strip : strips) {
            leds.add(strip);
            totalLength += strip.getLength();
        }

        led = new AddressableLED(0);
        led.setLength(160);


        buffer = new AddressableLEDBuffer(160);

        led.setData(buffer);
        led.start();
        set(State.BLACK);

    }

    public void setStripRed(int index, State s){
        // setStripSolid(leds.get(index), s);
        for(int i = 0; i < 160; i++){
            buffer.setLED(i, new Color(255, 0, 0));
        }

        led.setData(buffer);

    }

    public void setStripGreen(int index, State s){
        // setStripSolid(leds.get(index), s);
        for(int i = 0; i < 160; i++){
            buffer.setLED(i, new Color(0, 0, 255));
        }

        led.setData(buffer);

    }

    public void setStripSolid(LEDStrip strip, State s){
        Patterns.solid(strip, buffer, s.color);
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
}

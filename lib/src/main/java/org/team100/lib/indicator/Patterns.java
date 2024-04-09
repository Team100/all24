package org.team100.lib.indicator;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Patterns {


    /**
     * work-in-progress
     */
    static void rainbow(LEDStrip strip, AddressableLEDBuffer buffer) {
        int m_rainbowFirstPixelHue = 0;
        // For every pixel
        for (var i = strip.start(); i < strip.end(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    /**
     * work-in-progress
     */
    static void chase(Color[] colors, LEDStrip strip, AddressableLEDBuffer buffer) {
        int numberOfColors = colors.length;
        int effectiveIndex;
        int colorIndex;
        int offset = 0;
        int bufferLength = buffer.getLength();
        int segmentWidth = 0;

        for (int index = 0; index < bufferLength; index++) {
            effectiveIndex = (index + offset) % bufferLength;
            colorIndex = (index / segmentWidth) % numberOfColors;
            buffer.setLED(effectiveIndex, colors[colorIndex]);
        }

        offset = (offset + 1) % bufferLength;

    }

    private Patterns() {
        //
    }
}

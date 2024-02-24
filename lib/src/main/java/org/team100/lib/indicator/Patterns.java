// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.indicator;

import org.team100.lib.indicator.LEDIndicator.State;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class Patterns {


     public static void solid(LEDStrip strip, AddressableLEDBuffer buffer, Color s){
        for(var i = strip.getOffset(); i < strip.getLength(); i++){
            buffer.setLED(i, s);
        }
     }

    public static void rainbow(LEDStrip strip, AddressableLEDBuffer buffer) {

        int m_rainbowFirstPixelHue = 0;

        // For every pixel
        for (var i = strip.getOffset(); i < strip.getLength(); i++) {
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

      public static void chase(Color[] colors, LEDStrip strip, AddressableLEDBuffer buffer) {

        int numberOfColors = colors.length;
		int effectiveIndex;
		int colorIndex;
        int offset = 0;
		int bufferLength = buffer.getLength();
        int segmentWidth = 0;
        
		for (int index = 0; index < bufferLength; index++){
			effectiveIndex = (index + offset) % bufferLength;
			colorIndex =( index /segmentWidth )% numberOfColors;
			buffer.setLED(effectiveIndex, colors[colorIndex]);
		}

		offset =(offset+1) %bufferLength;

      }
}

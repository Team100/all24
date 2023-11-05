package org.team100;

import org.team100.optical_flow.PMW3901;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    // This creates an instance of the motion sensor.
    private final PMW3901 motion;

    // These accumulate sensor counts.
    private int accumX;
    private int accumY;

    public Robot() {
        motion = new PMW3901(SPI.Port.kOnboardCS0, 3);
        System.out.printf("*** Flow Sensor detected: %B\n", motion.isGoodSensor());
        if (!motion.isGoodSensor()) {
            System.out.println("Motion Sensor Initialization failed!");
        }
    }

    @Override
    public void teleopPeriodic() {
        PMW3901.Motion result = motion.readMotionCount();
        if (result != null) {
            accumX += result.getDeltaX();
            accumY += result.getDeltaY();
            System.out.printf(" %6d\t%6d\t%6d\t%6d\t%6d\t%6d\n",
            result.getDeltaX(), result.getDeltaY(), accumX, accumY, result.getSqual(), result.getShutter());
        }
    }
}

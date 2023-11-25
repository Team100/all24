
package org.team100.frc2024;

import org.team100.lib.barcode.Barcode;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Barcode barcode;

    public Robot() {
        barcode = new Barcode();
    }

    @Override
    public void robotInit() {
        barcode.robotInit();
    }

    @Override
    public void teleopPeriodic() {
        barcode.teleopPeriodic();
    }

    @Override
    public void testPeriodic() {
        barcode.testPeriodic();
    }

    @Override
    public void close() {
        super.close();
        barcode.close();
    }
}

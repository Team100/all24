package frc.robot;

import org.team100.lib.hid.ControlFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.RealFlight;

import edu.wpi.first.math.geometry.Twist2d;

public class RobotContainer {
    // private final DriverControl control;
    private final RealFlight control;

    public RobotContainer() {
        control = new RealFlight();
        DriverControl driverControl = new ControlFactory().getDriverControl();
    }

    public void periodic() {
        Twist2d twist = control.twist();

        System.out.printf("%5.3f %5.3f %5.3f\n",
                twist.dx, twist.dy, twist.dtheta);
    }
}

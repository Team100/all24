package frc.robot;

import org.team100.lib.hid.ControlFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.RealFlight;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
    private final DriverControl control;

    public RobotContainer() {
        control = new ControlFactory().getDriverControl();
        control.driveSlow(new PrintCommand("SLOW"));
        control.driveMedium(new PrintCommand("MEDIUM"));
        control.resetPose(new PrintCommand("RESET"));
    }

    public void periodic() {
        Twist2d twist = control.twist();

        System.out.printf("%5.3f %5.3f %5.3f\n",
                twist.dx, twist.dy, twist.dtheta);
    }

    public void print() {
        RealFlight rfcontrol = new RealFlight();
        Twist2d twist = rfcontrol.twist();

        double throttle = rfcontrol.throttle();
        System.out.printf("%5.3f %5.3f %5.3f %5.3f %s %s\n",
                twist.dx, twist.dy, twist.dtheta, throttle,
                rfcontrol.slow(), rfcontrol.medium()); 
    }
}

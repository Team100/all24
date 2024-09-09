package frc.robot;

import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

public class RobotContainer {
    final SupplierLogger demoLogger;

    public RobotContainer() {
        final Telemetry telemetry = Telemetry.get();
        demoLogger = telemetry.namedRootLogger("DEMO", true, false);
    }

    public void periodic() {
        demoLogger.logDouble(Level.COMP, "sine", () -> Math.sin(Timer.getFPGATimestamp()));
    }

}

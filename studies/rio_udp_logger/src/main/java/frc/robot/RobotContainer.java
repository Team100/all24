package frc.robot;

import java.util.ArrayList;
import java.util.List;

// import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2;
import org.team100.lib.telemetry.SupplierLogger2.DoubleSupplierLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.UdpPrimitiveLogger2;
import org.team100.lib.telemetry.UdpSender;

import edu.wpi.first.wpilibj.Timer;

public class RobotContainer {
    // about 10000 keys seems like the upper limit at 50hz: 500000 keys per second
    // works fine on localhost.
    private static final int logger_count = 10000;
    // final SupplierLogger demoLogger;
    final UdpPrimitiveLogger2 udpLogger;
    final List<DoubleSupplierLogger> doubleLogger = new ArrayList<>();

    public RobotContainer() {
        // final Telemetry telemetry = Telemetry.get();
        // demoLogger = telemetry.namedRootLogger("DEMO", true, false);

        udpLogger = new UdpPrimitiveLogger2(UdpSender.data(), UdpSender.meta());
        SupplierLogger2 logger = new SupplierLogger2(Telemetry.get(), "root", udpLogger);

        for (int i = 0; i < logger_count; ++i) {
            doubleLogger.add(logger.doubleLogger(Level.COMP, "sine" + i));
        }

    }

    public void periodic() {
        // demoLogger.logDouble(Level.COMP, "sine", () ->
        // Math.sin(Timer.getFPGATimestamp()));
        for (DoubleSupplierLogger l : doubleLogger) {
            l.log(() -> Math.sin(Timer.getFPGATimestamp()));
        }
        udpLogger.flush();
    }

}

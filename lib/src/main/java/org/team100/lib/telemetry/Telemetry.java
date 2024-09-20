package org.team100.lib.telemetry;

import org.team100.lib.logging.DataLogLogger;
import org.team100.lib.logging.DummySender;
import org.team100.lib.logging.NTLogger;
import org.team100.lib.logging.PrimitiveLogger;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.UdpPrimitiveLogger;
import org.team100.lib.logging.UdpPrimitiveLogger2;
import org.team100.lib.util.Util;

import com.ctre.phoenix6.SignalLogger;

/**
 * Simple logging wrapper.
 * 
 * Use keys of the form "/foo/bar"; the slashes separate levels in the tree.
 * 
 * Don't use a slash for any other reason, e.g. for "meters per second" don't
 * say "m/s" say "m_s"
 * 
 * Logged items are "retained" which means they persist even after the logging
 * stops; this means you should see the latest values after disabling the robot.
 */
public class Telemetry {
    /**
     * Using an experiment would be a pain.
     * TODO: remove the complexity here after the UDP thing is validated.
     */
    private static final boolean USE_UDP_LOGGING = true;

    public enum Level {
        /**
         * Minimal, curated set of measurements for competition matches. Comp mode
         * overrides root logger enablement.
         */
        COMP(1),
        /**
         * Keep this set small enough to avoid overrunning. This should only contain
         * things you're actively working on.
         */
        DEBUG(2),
        /**
         * Slow, shows absolutely everything. Overruns. Don't expect normal robot
         * behavior in this mode.
         */
        TRACE(3);

        private int priority;

        private Level(int priority) {
            this.priority = priority;
        }

        public boolean admit(Level other) {
            return this.priority >= other.priority;
        }
    }

    /**
     * Logging is prevented after this much time per cycle.
     * TODO: tune this value to leave time for real work.
     */
    private static final double kLoggingTimeBudgetS = 0.01;
    private static final Telemetry instance = new Telemetry();

    // private final NetworkTableInstance inst;
    final PrimitiveLogger ntLogger;
    final PrimitiveLogger usbLogger;
    final UdpPrimitiveLogger2 udpLogger;

    private Level m_level;

    /**
     * Uses the default network table instance.
     * Clients should use the static instance, not the constructor.
     */
    private Telemetry() {
        if (USE_UDP_LOGGING) {
            Util.warn("=======================================");
            Util.warn("Using UDP network logging!");
            Util.warn("You must have a log listener connected!");
            Util.warn("=======================================");
        }
        // inst = NetworkTableInstance.getDefault();
        ntLogger = new NTLogger();
        usbLogger = new DataLogLogger();
        // TODO: real senders
        DummySender dataSink = new DummySender();
        DummySender metaSink = new DummySender();

        udpLogger = new UdpPrimitiveLogger2(dataSink, metaSink);
        // this will be overridden by {@link TelemetryLevelPoller}
        m_level = Level.TRACE;
        // DataLogManager.start();
        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public void periodic() {
        udpLogger.periodic();
    }

    void setLevel(Level level) {
        m_level = level;
    }

    public Level getLevel() {
        return m_level;
    }

    public static Telemetry get() {
        return instance;
    }

    public SupplierLogger2 fieldLogger() {
        if (USE_UDP_LOGGING) {
            SupplierLogger2 logger = new NetworkLogger(this, "field");
            logger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");
            return logger;

        } else {
            return new FieldLogger(this);
        }
    }

    public SupplierLogger2 namedRootLogger(
            String str) {
        if (USE_UDP_LOGGING) {
            return new NetworkLogger(this, str);
        } else {
            return new RootLogger(this, str);
        }
    }

}
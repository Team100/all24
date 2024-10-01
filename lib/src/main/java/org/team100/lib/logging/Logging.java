package org.team100.lib.logging;

import org.team100.lib.logging.primitive.DummySender;
import org.team100.lib.logging.primitive.NTPrimitiveLogger2;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.logging.primitive.UdpPrimitiveLogger2;
import org.team100.lib.logging.primitive.UdpSender;
import org.team100.lib.util.Util;

import com.ctre.phoenix6.SignalLogger;

/** Logging singleton */
public class Logging {
    private static final boolean USE_UDP_LOGGING = false;
    private static final boolean USE_REAL_UDP = false;

    private static final Logging instance = new Logging();

    private UdpPrimitiveLogger2 udpLogger;
    private PrimitiveLogger ntLogger;
    private Level m_level;

    /**
     * root is "field", with a ".type"->"Field2d" entry as required by glass.
     */
    public final LoggerFactory fieldLogger;
    /** root is "log". */
    public final LoggerFactory rootLogger;

    /**
     * Clients should use the static instance, not the constructor.
     */
    private Logging() {
        // this will be overridden by {@link LogLevelPoller}
        m_level = Level.COMP;
        if (USE_UDP_LOGGING) {
            Util.warn("=======================================");
            Util.warn("Using UDP network logging!");
            Util.warn("You must have a log listener connected!");
            Util.warn("=======================================");
            if (USE_REAL_UDP) {
                udpLogger = new UdpPrimitiveLogger2(
                        UdpSender.data(),
                        UdpSender.meta());
            } else {
                udpLogger = new UdpPrimitiveLogger2(
                        new DummySender(),
                        new DummySender());
            }
            fieldLogger = new LoggerFactory(() -> m_level, "field", udpLogger);
            rootLogger = new LoggerFactory(() -> m_level, "log", udpLogger);
        } else {
            ntLogger = new NTPrimitiveLogger2();
            fieldLogger = new LoggerFactory(() -> m_level, "field", ntLogger);
            rootLogger = new LoggerFactory(() -> m_level, "log", ntLogger);
        }

        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");

        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public int keyCount() {
        if (udpLogger != null)
            return udpLogger.keyCount();
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    public void periodic() {
        if (udpLogger != null)
            udpLogger.periodic();
    }

    public void setLevel(Level level) {
        m_level = level;
    }

    public static Logging instance() {
        return instance;
    }
}
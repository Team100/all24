package org.team100.lib.logging;

import java.util.function.Supplier;

import org.team100.lib.logging.primitive.PrimitiveLogger;

public class FieldLogger extends LoggerFactory {
    /** Allow shared field logging. */
    public static class Log {
        public final DoubleArrayLogger m_log_target;
        public final DoubleArrayLogger m_log_ball;

        public Log(LoggerFactory log) {
            // Glass widget config remembers these names.
            // this name should become "/field/target".
            m_log_target = log.doubleArrayLogger(Level.TRACE, "target");
            // this name should become "/field/ball".
            m_log_ball = log.doubleArrayLogger(Level.TRACE, "ball");
        }
    }

    public FieldLogger(
            Supplier<Level> level,
            String root,
            PrimitiveLogger primitiveLoggerA) {
        super(level, root, primitiveLoggerA);
    }

}

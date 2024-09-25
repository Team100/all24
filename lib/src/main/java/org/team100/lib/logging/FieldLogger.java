package org.team100.lib.logging;

import java.util.function.Supplier;

import org.team100.lib.telemetry.Telemetry.Level;

public class FieldLogger extends SupplierLogger2 {
    /** Allow shared field logging. */
    public static class Log {
        public final DoubleArraySupplierLogger2 m_log_target;
        public final DoubleArraySupplierLogger2 m_log_ball;

        public Log(SupplierLogger2 log) {
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
            PrimitiveLogger2 primitiveLoggerA) {
        super(level, root, primitiveLoggerA);
    }

}

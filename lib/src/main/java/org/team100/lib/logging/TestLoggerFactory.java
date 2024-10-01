package org.team100.lib.logging;

import org.team100.lib.logging.primitive.PrimitiveLogger;

public class TestLoggerFactory extends LoggerFactory {

    public TestLoggerFactory(PrimitiveLogger primitiveLogger) {
        super(() -> Level.TRACE, "test", primitiveLogger);
    }

}

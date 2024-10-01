package org.team100.lib.logging;

import org.team100.lib.logging.primitive.PrimitiveLogger;

public class TestSupplierLogger extends LoggerFactory {

    public TestSupplierLogger(PrimitiveLogger primitiveLogger) {
        super(() -> Level.TRACE, "test", primitiveLogger);
    }

}

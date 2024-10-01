package org.team100.lib.logging;

import org.team100.lib.logging.primitive.PrimitiveLogger2;

public class TestSupplierLogger extends SupplierLogger2 {

    public TestSupplierLogger(PrimitiveLogger2 primitiveLogger) {
        super(() -> Level.TRACE, "test", primitiveLogger);
    }

}

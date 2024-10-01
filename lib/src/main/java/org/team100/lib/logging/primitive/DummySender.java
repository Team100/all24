package org.team100.lib.logging.primitive;

import java.nio.ByteBuffer;
import java.util.function.Consumer;

/**
 * For performance testing, to count output packets.
 */
public class DummySender implements Consumer<ByteBuffer> {
    private int m_counter = 0;

    @Override
    public void accept(ByteBuffer arg0) {
        m_counter++;
    }

    public int getCounter() {
        return m_counter;
    }
}
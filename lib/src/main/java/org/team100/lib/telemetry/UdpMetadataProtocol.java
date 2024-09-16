package org.team100.lib.telemetry;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.time.Instant;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Log metadata protocol
 * 
 * Metadata packets are lists of tuples:
 * 
 * * key (2 bytes)
 * * type (1 byte)
 * * label (1 byte length + ascii string)
 * 
 * I previously had a more complicated, terse, stateful protocol, but I think
 * it's worth a few bytes to be simpler.
 * 
 * This protocol is very simple: input tuples, write them to a buffer.
 * 
 * The protocol itself doesn't enforce singlevaluedness (i.e. one label per
 * key); the caller should do that.
 * 
 * <pre>
 * DDDDDDDDKKTLAAAAKKTLAAAAAKKTLAAA
 * ^^^^^^^^                          timestamp
 *         ^^                        key = 16
 *           ^                       type = 5 (int)
 *            ^                      string length = 4
 *             ^^^^                  string in ascii for label 16
 *                 ^^                key = 17
 *                   ^               type = 5 (int) 
 *                    ^              string length = 5
 *                     ^^^^^         string in ascii for label 17
 *                          ^^       key = 18
 *                            ^      type = 3 (bool)
 *                             ^     string length = 3
 *                              ^^^  string in ascii for label 18
 * </pre>
 */
public class UdpMetadataProtocol {
    /**
     * Epoch seconds timestamp is used as the version key for the label map.
     * It is provided by the driver station, so it is not available at startup,
     * instead, you check the value periodically and set it when it becomes
     * available.
     */
    static long timestamp;

    private final ByteBuffer m_buffer;

    public UdpMetadataProtocol() {
        m_buffer = ByteBuffer.allocate(508);
        // this is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
        m_buffer.putLong(timestamp); // timetstamp = 8 bytes
    }

    /** Return a buffer view of length equal to current position. */
    ByteBuffer trim() {
        return m_buffer.slice(0, m_buffer.position());
    }

    /** for testing */
    ByteBuffer buffer() {
        return m_buffer;
    }

    /**
     * If it hasn't already been set, and if a valid value is available, set the
     * timestamp.
     */
    private static void setTimestamp() {
        if (timestamp != 0)
            return; // already set
        if (!RobotController.isSystemTimeValid())
            return; // no time available
        timestamp = Instant.now().getEpochSecond();
    }

    boolean put(int key, UdpType type, String label) {
        return add(m_buffer, key, type, label);
    }

    /**
     * <pre>
     * KKTLAAAA
     * ^^       key = 16
     *   ^      type = 5 (int)
     *    ^     string length = 4
     *     ^^^^ string in ascii for label 16
     * </pre>
     * 
     * @return true if written
     */
    static boolean add(ByteBuffer buf, int key, UdpType type, String label) {
        if (key < 16)
            throw new IllegalArgumentException("key in type code range: " + key);
        if (key > 65535)
            throw new IllegalArgumentException("key too large");
        byte[] bytes = label.getBytes(StandardCharsets.US_ASCII);
        if (bytes.length > 255)
            throw new IllegalArgumentException("label too long");
        int n = bytes.length;
        if (4 + n > buf.remaining())
            return false;
        buf.putChar((char) key); // key = 2 bytes
        buf.put(type.id); // type = 1 byte
        buf.put((byte) n); // length = 1 byte
        buf.put(bytes); // string = N bytes
        return true;
    }
}

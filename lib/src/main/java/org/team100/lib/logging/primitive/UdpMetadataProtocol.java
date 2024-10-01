package org.team100.lib.logging.primitive;

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
 *           ^                       type = 3 (int)
 *            ^                      string length = 4
 *             ^^^^                  string in ascii for label 16
 *                 ^^                key = 17
 *                   ^               type = 3 (int) 
 *                    ^              string length = 5
 *                     ^^^^^         string in ascii for label 17
 *                          ^^       key = 18
 *                            ^      type = 1 (bool)
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

    public UdpMetadataProtocol(int bufferSize) {
        // direct buffer goes slightly faster out the network
        m_buffer = ByteBuffer.allocateDirect(bufferSize);
        // big-endian is the default, but just to make it clear...
        m_buffer.order(ByteOrder.BIG_ENDIAN);
        m_buffer.putLong(timestamp); // timetstamp = 8 bytes
    }

    public UdpMetadataProtocol() {
        this(UdpSender.MTU);
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
     * Clear the underlying buffer, update the timestamp if possible, and write the
     * timestamp into the buffer.
     * 
     * TODO: allow setting the timestamp to a specific value, for testing.
     */
    void clear() {
        m_buffer.clear();
        setTimestamp();
        m_buffer.putLong(UdpMetadataProtocol.timestamp);
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
     * caller should check key and label sizes.
     * TODO: use bytes for label to save a little time
     * 
     * @return true if written
     */
    static boolean add(ByteBuffer buf, int key, UdpType type, String label) {
        byte[] bytes = label.getBytes(StandardCharsets.US_ASCII);
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

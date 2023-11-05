package org.team100.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

/**
 * The TI ADS1015 is a 12-bit 4-channel ADC with I2C interface.
 * 
 * This is mostly cribbed from SparkFun and Adafruit:
 * 
 * https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library
 * https://github.com/adafruit/Adafruit_ADS1X15
 * 
 * Sections and Tables referenced below are from the datasheet:
 * 
 * https://www.ti.com/lit/ds/symlink/ads1015.pdf
 * 
 * Notable from the datasheet: ADS1015 is incapable of clock stretching (section
 * 9.1.1), so maybe it will work fine on the MXP I2C port?
 * 
 * The ADS1015 contains a single ADC and a 4-way mux, so reading involves
 * configuring the mux, starting the measurement, waiting for completion, and
 * then reading the result.
 */
public class ADS1015 {
    /**
     * Operational status.
     * 
     * See Section 8.6.3, Table 6, bit 15, when writing.
     */
    public enum OS_W {
        NOP(0b0000_0000_0000_0000),
        START(0b1000_0000_0000_0000);

        private final short value;

        private OS_W(int value) {
            this.value = (short) value;
        }
    }

    /**
     * Start a single conversion.
     * 
     * See Section 8.6.3, Table 6, bit 15, when reading.
     */
    public enum OS_R {
        BUSY(0b0000_0000_0000_0000),
        NOT_BUSY(0b1000_0000_0000_0000);

        private final short value;
        private static final short mask = (short) 0b1000_0000_0000_0000;

        private OS_R(int value) {
            this.value = (short) value;
        }
    }

    /**
     * Input multiplexer configuration.
     * 
     * See Section 8.6.3, Table 6, bits 14:12.
     */
    public enum MUX {
        SINGLE_0(0b0100_0000_0000_0000),
        SINGLE_1(0b0101_0000_0000_0000),
        SINGLE_2(0b0110_0000_0000_0000),
        SINGLE_3(0b0111_0000_0000_0000);

        private static final MUX[] values = new MUX[] {
                SINGLE_0,
                SINGLE_1,
                SINGLE_2,
                SINGLE_3
        };
        private final short value;

        private MUX(int value) {
            this.value = (short) value;
        }

        public static MUX get(int channel) {
            return values[channel];
        }
    }

    /**
     * Programmable gain amplifier configuration.
     * 
     * See Section 8.6.3, Table 6, bits 11:9.
     */
    public enum PGA {
        FSR_6_144V(0b0000_0000_0000_0000, 3),
        FSR_4_096V(0b0000_0010_0000_0000, 2),
        FSR_2_048V(0b0000_0100_0000_0000, 1),
        FSR_1_024V(0b0000_0110_0000_0000, 0.5),
        FSR_0_512V(0b0000_1000_0000_0000, 0.25),
        FSR_0_256V(0b0000_1010_0000_0000, 0.125);

        private final short value;
        /**
         * Millivolts per bit.  See Section 8.3.3, Table 1.
         */
        private final double mV;

        private PGA(int value, double mV) {
            this.value = (short) value;
            this.mV = mV;
        }
    }

    /**
     * Device operating mode.
     * 
     * See Section 8.6.3, Table 6, bit 8.
     */
    public enum MODE {
        CONTINUOUS(0b0000_0000_0000_0000),
        SINGLE(0b0000_0001_0000_0000);

        private final short value;

        private MODE(int value) {
            this.value = (short) value;
        }
    }

    /**
     * Data rate.
     * 
     * See Section 8.6.3, Table 6, bits 7:5
     */
    public enum DR {
        SPS_128(0b0000_0000_0000_0000, 128),
        SPS_250(0b0000_0000_0010_0000, 250),
        SPS_490(0b0000_0000_0100_0000, 490),
        SPS_920(0b0000_0000_0110_0000, 920),
        SPS_1600(0b0000_0000_1000_0000, 1600),
        SPS_2400(0b0000_0000_1010_0000, 2400),
        SPS_3300(0b0000_0000_1100_0000, 3300);

        private final short value;
        private final int sps;

        private DR(int value, int sps) {
            this.value = (short) value;
            this.sps = sps;
        }
    }

    /**
     * I2C address.
     * 
     * 7-bit addr is 0x48, 8-bit addr is 0x91
     */
    private static final byte ADDR = (byte) 0x91;

    /**
     * Conversion register.
     * 
     * Contains the 12-bit conversion result, left justified in 16 bits.
     * 
     * See Section 8.6.2.
     */
    private static final byte CONV_REG = (byte) 0x00;

    /**
     * Config register.
     * 
     * Use the enums above to set bits in this register.
     * 
     * Bits 4:0 are unused, they pertain to the comparator function which sets
     * physical pins.
     * 
     * See Section 8.6.3.
     */
    private static final byte CFG_REG = (byte) 0x01;

    private final I2C m_i2c;
    private final PGA m_pga;
    private final DR m_dr;

    /**
     * Appropriate for measuring 3.3v signals as slowly and accurately as possible.
     */
    public ADS1015() {
        this(ADDR, PGA.FSR_4_096V, DR.SPS_128);
    }

    private ADS1015(byte i2cAddress, PGA pga, DR dr) {
        m_i2c = new I2C(I2C.Port.kMXP, i2cAddress >>> 1);
        m_pga = pga;
        m_dr = dr;
    }

    /**
     * Read the voltage on the specified channel.
     * 
     * The full-scale is double-ended (plus and minus) so the single-ended
     * GND-referenced reading will use half of the 12-bit range.
     * 
     * Returns zero if something goes wrong (e.g. device not connected or takes
     * too long).
     * 
     * See Section 9.1.2 and Section 8.3.3.
     */
    public double readVolts(int channel) {
        return readRaw(channel) * m_pga.mV / 1000;
    }

    /**
     * Read the raw 12-bit ADC value for the specified channel.
     * 
     * Returns zero if something goes wrong (e.g. device not connected or takes too
     * long).
     */
    public short readRaw(int channel) {
        if (channel < 0 || channel > 3)
            throw new IllegalArgumentException(String.format("Illegal channel: %d", channel));
        startADCReading(MUX.get(channel));
        // Wait for the ADC to finish reading.
        // Try a few times (e.g. to recover from i2c bus collisions).
        int maxBusy = 5;
        do {
            try {
                // Single-shot power-up takes 25us. See Section 8.4.2.1.
                Thread.sleep(0, 25000);
                // Conversion time is the inverse of the data rate. See Section 8.3.6.
                Thread.sleep(1000 / m_dr.sps, 1000000000 / m_dr.sps % 1000000);
            } catch (InterruptedException e) {
                // keep trying
            }
            maxBusy -= 1;
            if (maxBusy < 0) {
                return 0;
            }
        } while (busy());

        return getLastConversionResult();
    }

    /**
     * Start a single-shot measurement and return immediately.
     */
    private void startADCReading(MUX mux) {
        short config = 0;
        config |= OS_W.START.value;
        config |= mux.value;
        config |= m_pga.value;
        config |= MODE.SINGLE.value;
        config |= m_dr.value;
        writeRegister(CFG_REG, config);
    }

    /**
     * True if the device is currently doing a measurement.
     */
    private boolean busy() {
        return (readRegister(CFG_REG) & OS_R.mask) == OS_R.BUSY.value;
    }

    /**
     * Read the result.
     * 
     * The ADS1015 reads 12 bits into 16 left justified, so this shifts right by 4.
     */
    private short getLastConversionResult() {
        return (short) (readRegister(CONV_REG) >>> 4);
    }

    /**
     * Read two bytes from the specified register.
     */
    private short readRegister(byte register) {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(register, 2, buf);
        return buf.getShort();
    }

    /**
     * Write two bytes to the specified register
     */
    private void writeRegister(byte register, short value) {
        ByteBuffer buf = ByteBuffer.allocate(3);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        buf.put(register);
        buf.putShort(value);
        m_i2c.writeBulk(buf, 3);
    }
}

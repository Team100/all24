package org.team100.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;

/**
 * Adafruit 4517 contains this gyro, interfaced with I2C.
 * 
 * (It is also possible to use SPI with this device but this class supports only I2C.)
 * 
 * It uses NWU coordinates, clockwise-negative, like most of WPILIB.
 * 
 * Full datasheet is at https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
 */
public class LSM6DSOX_I2C {
    /**
     * Gyro output Data Rate (ODR).
     * 
     * The output rate affects the response of the sensor, see Table 18.
     * We are effectively sampling the output at 50Hz, so to avoid aliasing
     * we need to remove any signal above 25Hz, so we should choose the 52Hz
     * ODR.
     * 
     * These are the high 4 bits in CTRL2_G, the gyro control register.
     * 
     * This list duplicates Table 55.
     */
    public enum ODR_G {
        ODR_OFF(0b0000_0000),
        ODR_12Hz5(0b0001_0000),
        ODR_26Hz(0b0010_0000),
        ODR_52Hz(0b0011_0000),
        ODR_104Hz(0b0100_0000),
        ODR_208Hz(0b0101_0000),
        ODR_417Hz(0b0110_0000),
        ODR_833Hz(0b0111_0000),
        ODR_1667Hz(0b1000_0000),
        ODR_3333Hz(0b1001_0000),
        ODR_6667Hz(0b1010_0000);

        /**
         * Mask to erase the relevant bits.
         */
        private static final byte kMask = (byte) 0b1111_0000;

        /**
         * Register value
         */
        private final byte value;

        private ODR_G(int value) {
            this.value = (byte) value;
        }

        /**
         * Read the ctrl register, modify the odr bits, write it back.
         */
        private void set(I2C m_i2c) {
            ByteBuffer buf = ByteBuffer.allocate(1);
            buf.order(ByteOrder.LITTLE_ENDIAN);
            m_i2c.read(CTRL2_G, 1, buf);
            byte ctrl2 = buf.get();
            ctrl2 &= ~ODR_G.kMask;
            ctrl2 |= value;
            m_i2c.write(CTRL2_G, ctrl2);
        }
    }

    /**
     * Gyro sensitivity.
     * 
     * This represents the "full-scale" output of the gyro in degrees per second.
     * The choice of full-scale also affects the discretization error, but it's
     * a (signed) 16-bit output so there's a lot of headroom. It's hard to imagine
     * a robot moving more than 500 degrees per second
     * 
     * These are bits 1-3 in CTRL2_G, the gyro control register.
     * 
     * This list duplicates Table 54.
     */
    public enum FS_G {
        FS_125dps(0b0000_0010, 4.375),
        FS_250dps(0b0000_0000, 8.75),
        FS_500dps(0b0000_0100, 17.5),
        FS_1000dps(0b0000_1000, 35),
        FS_2000dps(0b0000_1100, 70);

        /**
         * Register value.
         */
        private final byte value;
        /**
         * Millidegrees per second per bit
         */
        private final double mdps;
        /**
         * Mask to erase the relevant bits.
         */
        private static final byte mask = (byte) 0b0000_1110;

        private FS_G(int value, double mdps) {
            this.value = (byte) value;
            this.mdps = mdps;
        }

        /**
         * Read the ctrl register, modify the scale bits, write it back.
         */
        private void set(I2C m_i2c) {
            ByteBuffer buf = ByteBuffer.allocate(1);
            buf.order(ByteOrder.LITTLE_ENDIAN);
            m_i2c.read(CTRL2_G, 1, buf);
            byte ctrl2 = buf.get();
            ctrl2 &= ~FS_G.mask;
            ctrl2 |= value;
            m_i2c.write(CTRL2_G, ctrl2);
        }
    }

    /**
     * I2C address. 8-bit addr is 0xd5, so 7-bit is shifted, 0x6A
     */
    private static final byte ADDR = (byte) 0xD5;
    /**
     * Control register for scale and rate. See datasheet section 9.16.
     */
    private static final byte CTRL2_G = (byte) 0x11;

    /**
     * Output register, little-endian, 16b.  See datasheet section 9.33.
     */
    private static final byte OUTZ_L_G = (byte) 0x26;

    /**
     * Static offset.
     * updated for "minibotNEO".
     * TODO: different offset for each robot
     * TODO: capture offset at startup, when motionless.
     */
    private static final int kRawOffset = -16;

    private final I2C m_i2c;
    private final FS_G m_scale;
    private final DoublePublisher measurementPub;
    private final DoublePublisher measurementRawPub;

    /**
     * Construct the gyro with sensitivity of 500 degrees/sec and 52 Hz data rate.
     */
    public LSM6DSOX_I2C() {
        this(ADDR,
                ODR_G.ODR_52Hz,
                FS_G.FS_500dps);
    }

    /**
     * NWU yaw rate in radians/sec.
     */
    public double getYawRateRadS() {
        int yawRateRaw = getYawRateRaw();
        measurementRawPub.set(yawRateRaw);
        double yawRateRadS = yawRateRaw * m_scale.mdps * Math.PI / 180000;
        measurementPub.set(yawRateRadS);
        return yawRateRadS;
    }

    //////////////////////////////////////////////////////////////////////////

    private LSM6DSOX_I2C(
            byte i2cAddress,
            ODR_G odr,
            FS_G fs) {
        m_i2c = new I2C(I2C.Port.kMXP, i2cAddress >>> 1);
        odr.set(m_i2c);
        m_scale = fs;
        fs.set(m_i2c);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("gyro");
        measurementPub = table.getDoubleTopic("yawRateRadS").publish();
        measurementRawPub = table.getDoubleTopic("yawRateRaw").publish();
    }

    /**
     * NWU yaw rate, 16 bits, signed. Unit depends on full-scale setting.
     */
    private int getYawRateRaw() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(OUTZ_L_G, 2, buf);
        return buf.getShort() - kRawOffset;
    }
}

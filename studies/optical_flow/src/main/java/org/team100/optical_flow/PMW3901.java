package org.team100.optical_flow;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class PMW3901 {
    public static class Motion {
        private final int deltaX;
        private final int deltaY;
        private final int squal;
        private final int shutter;

        public Motion(int deltaX, int deltaY, int squal, int shutter) {
            this.deltaX = deltaX;
            this.deltaY = deltaY;
            this.squal = squal;
            this.shutter = shutter;
        }

        public int getDeltaX() {
            return deltaX;
        }

        public int getDeltaY() {
            return deltaY;
        }

        public int getSqual() {
            return squal;
        }

        public int getShutter() {
            return shutter;
        }
    }

    private final SPI spiFlow;
    private final DigitalOutput cs;
    private final boolean goodSensor;

    public PMW3901(SPI.Port port, int chipSelectPort) {
        spiFlow = new SPI(port);
        cs = new DigitalOutput(chipSelectPort);
        goodSensor = initSequence(cs);
    }

    public Motion readMotionCount() {
        if (!goodSensor)
            return null;

        // Read the status register and test the motion bit to see if new data is
        // available.
        if ((registerRead((byte) 0x02) & 0x80) == 0x80) {
            // Read the rest of the data registers in numerical order, and convert to ints.
            byte dXL = registerRead((byte) 0x03);
            byte dXH = registerRead((byte) 0x04);
            byte dYL = registerRead((byte) 0x05);
            byte dYH = registerRead((byte) 0x06);

            int squal = 0 | registerRead((byte) 0x07);
            int shutter = 0 | registerRead((byte) 0x0C);
            int deltaX = (dXH << 8) | dXL & 0xff;
            int deltaY = (dYH << 8) | dYL & 0xff;

            if ((shutter == 31) && (squal < 25)) {
                System.out.printf("**** BAD READING FROM FLOW SENSOR ****");
                return null;
            }

            return new Motion(deltaX, deltaY, squal, shutter);
        } else {
            // No motion detected.
            return null;
        }
    }

    /**
     * Write a register via the SPI port.
     */
    private void registerWrite(byte reg, byte value) {
        // Data transfer buffer.
        byte[] flowdata = new byte[2];
        // Set the high order bit on the write register.
        reg |= (byte) 0x80;
        // Populate the buffer.
        flowdata[0] = reg;
        flowdata[1] = value;
        // Set chip select.
        cs.set(false);
        microSleep(50);
        // write the register address, and read the data.
        spiFlow.write(flowdata, 2);
        microSleep(50);
        // Clear chip select.
        cs.set(true);
        microSleep(200);
        // System.out.printf("registerWrite buffer = [0]%02X [1]%02X\n", flowdata[0],
        // flowdata[1]);
    }

    /**
     * Read a register byte via the SPI port.
     */
    private byte registerRead(byte reg) {
        // Data transfer buffer.
        byte[] flowdata = new byte[2];
        // Clear the high order bit on the read register.
        reg &= 0x7F;
        // Populate the buffer.
        flowdata[0] = reg;
        flowdata[1] = 0;
        // Set chip select.
        cs.set(false);
        microSleep(50);
        // write the reg address.
        spiFlow.write(flowdata, 1);
        microSleep(50);
        // read the register data.
        spiFlow.read(true, flowdata, 1);
        microSleep(200);

        // Clear chip select.
        cs.set(true);
        // System.out.printf("registerRead buffer = [0]%02X [1]%02X\n", flowdata.get(0),
        // flowdata.get(1));
        return flowdata[0];
    }

    /**
     * Initialize the SPI Port and the sensor chip.
     */
    private boolean initSequence(DigitalOutput cs) {
        // Configure these settings to match SPI Mode 3. (See Wikipedia)
        // i think this isn't actually mode 3?
        spiFlow.setClockRate(2000000); // 2 MHz
        // clock idle low, sample on rising edge
        spiFlow.setMode(SPI.Mode.kMode0);
        spiFlow.setChipSelectActiveLow();
        System.out.println("SPI port is initialized.");
        // Reset the sensor SPI bus.
        cs.set(true);
        Timer.delay(0.001);
        cs.set(false);
        Timer.delay(0.001);
        cs.set(true);
        Timer.delay(0.001);

        // Initialize the sensor chip.
        // Power on reset
        registerWrite((byte) 0x3A, (byte) 0x5A);
        Timer.delay(.005);

        // Test the SPI communications, checking chipId and inverse chipId
        byte Product_ID = registerRead((byte) 0x00);
        byte Inverse_Product_ID = registerRead((byte) 0x5F);
        System.out.printf("*** ChipID 0x49: %02X, InverseChipID 0xB6: %02X\n", Product_ID, Inverse_Product_ID);
        if (Product_ID != (byte) 0x49 && Inverse_Product_ID != (byte) 0xB6) {
            System.out.println("Sensor chip did not initialize.");
            return false;
        }
        // Reading the motion registers one time. The data isn't used.
        registerRead((byte) 0x02);
        registerRead((byte) 0x03);
        registerRead((byte) 0x04);
        registerRead((byte) 0x05);
        registerRead((byte) 0x06);
        Timer.delay(.001);

        // Initialize the chip's registers.
        registerWrite((byte) 0x7F, (byte) 0x00);
        registerWrite((byte) 0x61, (byte) 0xAD);
        registerWrite((byte) 0x7F, (byte) 0x03);
        registerWrite((byte) 0x40, (byte) 0x00);
        registerWrite((byte) 0x7F, (byte) 0x05);
        registerWrite((byte) 0x41, (byte) 0xB3);
        registerWrite((byte) 0x43, (byte) 0xF1);
        registerWrite((byte) 0x45, (byte) 0x14);
        registerWrite((byte) 0x5B, (byte) 0x32);
        registerWrite((byte) 0x5F, (byte) 0x34);
        registerWrite((byte) 0x7B, (byte) 0x08);
        registerWrite((byte) 0x7F, (byte) 0x06);
        registerWrite((byte) 0x44, (byte) 0x1B);
        registerWrite((byte) 0x40, (byte) 0xBF);
        registerWrite((byte) 0x4E, (byte) 0x3F);
        registerWrite((byte) 0x7F, (byte) 0x08);
        registerWrite((byte) 0x65, (byte) 0x20);
        registerWrite((byte) 0x6A, (byte) 0x18);
        registerWrite((byte) 0x7F, (byte) 0x09);
        registerWrite((byte) 0x4F, (byte) 0xAF);
        registerWrite((byte) 0x5F, (byte) 0x40);
        registerWrite((byte) 0x48, (byte) 0x80);
        registerWrite((byte) 0x49, (byte) 0x80);
        registerWrite((byte) 0x57, (byte) 0x77);
        registerWrite((byte) 0x60, (byte) 0x78);
        registerWrite((byte) 0x61, (byte) 0x78);
        registerWrite((byte) 0x62, (byte) 0x08);
        registerWrite((byte) 0x63, (byte) 0x50);
        registerWrite((byte) 0x7F, (byte) 0x0A);
        registerWrite((byte) 0x45, (byte) 0x60);
        registerWrite((byte) 0x7F, (byte) 0x00);
        registerWrite((byte) 0x4D, (byte) 0x11);
        registerWrite((byte) 0x55, (byte) 0x80);
        registerWrite((byte) 0x74, (byte) 0x1F);
        registerWrite((byte) 0x75, (byte) 0x1F);
        registerWrite((byte) 0x4A, (byte) 0x78);
        registerWrite((byte) 0x4B, (byte) 0x78);
        registerWrite((byte) 0x44, (byte) 0x08);
        registerWrite((byte) 0x45, (byte) 0x50);
        registerWrite((byte) 0x64, (byte) 0xFF);
        registerWrite((byte) 0x65, (byte) 0x1F);
        registerWrite((byte) 0x7F, (byte) 0x14);
        registerWrite((byte) 0x65, (byte) 0x67);
        registerWrite((byte) 0x66, (byte) 0x08);
        registerWrite((byte) 0x63, (byte) 0x70);
        registerWrite((byte) 0x7F, (byte) 0x15);
        registerWrite((byte) 0x48, (byte) 0x48);
        registerWrite((byte) 0x7F, (byte) 0x07);
        registerWrite((byte) 0x41, (byte) 0x0D);
        registerWrite((byte) 0x43, (byte) 0x14);
        registerWrite((byte) 0x4B, (byte) 0x0E);
        registerWrite((byte) 0x45, (byte) 0x0F);
        registerWrite((byte) 0x44, (byte) 0x42);
        registerWrite((byte) 0x4C, (byte) 0x80);
        registerWrite((byte) 0x7F, (byte) 0x10);
        registerWrite((byte) 0x5B, (byte) 0x02);
        registerWrite((byte) 0x7F, (byte) 0x07);
        registerWrite((byte) 0x40, (byte) 0x41);
        registerWrite((byte) 0x70, (byte) 0x00);
        Timer.delay(0.01);
        registerWrite((byte) 0x32, (byte) 0x44);
        registerWrite((byte) 0x7F, (byte) 0x07);
        registerWrite((byte) 0x40, (byte) 0x40);
        registerWrite((byte) 0x7F, (byte) 0x06);
        registerWrite((byte) 0x62, (byte) 0xf0);
        registerWrite((byte) 0x63, (byte) 0x00);
        registerWrite((byte) 0x7F, (byte) 0x0D);
        registerWrite((byte) 0x48, (byte) 0xC0);
        registerWrite((byte) 0x6F, (byte) 0xd5);
        registerWrite((byte) 0x7F, (byte) 0x00);
        registerWrite((byte) 0x5B, (byte) 0xa0);
        registerWrite((byte) 0x4E, (byte) 0xA8);
        registerWrite((byte) 0x5A, (byte) 0x50);
        registerWrite((byte) 0x40, (byte) 0x80);
        registerWrite((byte) 0x7F, (byte) 0x00);
        registerWrite((byte) 0x5A, (byte) 0x10);
        registerWrite((byte) 0x54, (byte) 0x00);
        System.out.println("Sensor chip is initialized.");
        return true;
    }

    private void microSleep(int usec) {
        try {
            Thread.sleep(0, usec * 1000);
        } catch (InterruptedException e) {
            // ignore this exception
        }
    }

    public boolean isGoodSensor() {
        return goodSensor;
    }
}
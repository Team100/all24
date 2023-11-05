package org.team100.lib.retro;

import org.team100.lib.config.Identity;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class Illuminator implements IlluminatorInterface {
    /** For robots without an illuminator. */
    private static class Noop implements IlluminatorInterface {

        @Override
        public void set(double i) {
            //
        }

        @Override
        public void close() {
            //
        }
    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public IlluminatorInterface get(int deviceId) {
            switch (m_identity) {
                case SWERVE_ONE:
                    return new Illuminator(deviceId);
                default:
                    return new Noop();
            }
        }
    }

    public static class Config {

        /**
         * There is no current-limiting resistor in this setup, we rely on voltage to
         * choose an output level; this has the advantage of simplicity and avoiding a
         * multi-watt resistor, but it has the disadvantage that the LED output is not
         * constant with voltage, it depends on temperature in a thermal-runaway
         * fashion. See https://assets.cree-led.com/a/ds/x/XLamp-XPE2.pdf page 3:
         * temperature coefficient of green is -1.2 mV/C, red is -1.8 mV/C.
         * 
         * Say the junction is 100 C so the forward voltage droops by about 0.2 V.
         * So keep the max voltage, say, 0.5 V away from the maximum to account
         * for heating.
         * 
         * But since the maximum is higher than the supply voltage, I think 12 is fine.
         */
        public double kMaxVoltage = 12.0;
    }

    private final Config m_config = new Config();
    private final CANSparkMax led;

    private Illuminator(int deviceId) {
        // note that "smart" current limiting appears to be for brushless motors only,
        // and "secondary" current limiting appears not to work (it produces an error in
        // the log, and happily outputs max current), so we don't use either one.
        led = new CANSparkMax(deviceId, MotorType.kBrushed);
        led.setInverted(true);
        String version = led.getFirmwareString();
        System.out.println("Illuminator controller version " + version);
    }

    /**
     * @param value in range [0,1]
     */
    @Override
    public void set(final double value) {
        double clampedValue = MathUtil.clamp(value, 0, 1);
        double outputVolts = clampedValue * m_config.kMaxVoltage;
        led.setVoltage(outputVolts);
    }

    @Override
    public void close() {
        led.close();
    }
}
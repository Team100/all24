package org.team100.lib.motion.example1d.sled;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SledTest {
    private static final double kDelta = 0.001;

    /**
     * Test the response of the simulated sled over time, as the actuator supplies
     * receives actuations.
     */
    @Test
    void testActuator() {
        SledVelocityServo servo = new SledVelocityServo(new SledActuation(0));
        SimulatedSled sled = new SimulatedSled(servo);

        {
            // motionless
            SledActuation actuation = new SledActuation(0);
            servo.set(actuation);
            sled.update(0.02);
            assertEquals(0, sled.getConfig().getPositionM(), kDelta);
            sled.update(0.02);
            assertEquals(0, sled.getConfig().getPositionM(), kDelta);
        }

        {
            // move at 1m/s
            SledActuation actuation = new SledActuation(1);
            servo.set(actuation);
            sled.update(0.02);
            assertEquals(0.02, sled.getConfig().getPositionM(), kDelta);
            sled.update(0.02);
            assertEquals(0.04, sled.getConfig().getPositionM(), kDelta);
        }
    }

    @Test
    void testConfigurationController() {
        SledVelocityServo servo = new SledVelocityServo(new SledActuation(0));
        SimulatedSled sled = new SimulatedSled(servo);
        SledConfigurationController confCon = new SledConfigurationController();
        SledConfiguration setpoint = new SledConfiguration(0);
        SledConfiguration measurement = sled.getConfig();
        SledActuation actuation = confCon.calculate(measurement, setpoint);


    }

}

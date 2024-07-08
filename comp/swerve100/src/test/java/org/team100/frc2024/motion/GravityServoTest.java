package org.team100.frc2024.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.TestLogger24;
import org.team100.frc2024.Timeless2024;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Logger;

import edu.wpi.first.math.controller.PIDController;

class GravityServoTest implements Timeless2024 {
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger24();

    @Test
    void testSetPosition() {
        PIDController pivotController = new PIDController(4.5, 0.0, 0.000);
        TrapezoidProfile100 profile = new TrapezoidProfile100(8, 8, 0.001);
        double period = 0.02;
        // motor speed is rad/s
        SimulatedBareMotor simMotor = new SimulatedBareMotor(logger, 600);
        RotaryMechanism simMech = new RotaryMechanism(simMotor, 165);
        SimulatedRotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                logger,
                simMech);

        GravityServo g = new GravityServo(
                simMech,
                logger,
                pivotController,
                profile,
                period,
                simEncoder);
        // start at zero
        assertEquals(0, g.getPositionRad().getAsDouble(), kDelta);
        // one second
        for (int i = 0; i < 50; ++i) {
            g.setPosition(1);
            stepTime(0.02);
        }
        // this overshoots a little, i think maybe because of the slight lag in
        // measurement.
        assertEquals(1, g.getPositionRad().getAsDouble(), kDelta);
    }

}

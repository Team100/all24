package org.team100.frc2024.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.Timeless2024;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.GravityServoInterface;

import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

import edu.wpi.first.math.controller.PIDController;

class GravityServoTest implements Timeless2024 {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSetPosition() {
        double period = 0.02;
        PIDController pivotController = new PIDController(4.5, 0.0, 0.000, period);
        TrapezoidProfile100 profile = new TrapezoidProfile100(8, 8, 0.001);
        // motor speed is rad/s
        SimulatedBareMotor simMotor = new SimulatedBareMotor(logger, 600);
        RotaryMechanism simMech = new SimpleRotaryMechanism(
                logger,
                simMotor,
                new SimulatedBareEncoder(logger, simMotor),
                165);
        SimulatedRotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                logger,
                simMech);


        AngularPositionServo servo = new OnboardAngularPositionServo(
                logger,
                simMech,
                simEncoder,
                () -> profile,
                pivotController);
        servo.reset();

        GravityServoInterface g = new OutboardGravityServo(
                servo, 5.0, 0.0);
        // start at zero
        assertEquals(0, g.getPositionRad().getAsDouble(), kDelta);
        // one second
        for (int i = 0; i < 70; ++i) {
            g.setPosition(1);
            stepTime(0.02);
            // System.out.printf("%5.3f\n", g.getPositionRad().getAsDouble());

        }
        // this overshoots a little, i think maybe because of the slight lag in
        // measurement.
        assertEquals(1.0004, g.getPositionRad().getAsDouble(), kDelta);
    }
}

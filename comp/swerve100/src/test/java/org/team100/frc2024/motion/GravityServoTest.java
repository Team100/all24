package org.team100.frc2024.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.Timeless2024;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

class GravityServoTest implements Timeless2024 {
    private static final double kDelta = 0.001;

    @Test
    void testSetPosition() {

        SysParam pivotParams = SysParam.neoPositionServoSystem(
                165, // gear ratio
                300, // max vel
                300); // max accel

        PIDController pivotController = new PIDController(4.5, 0.0, 0.000);
        TrapezoidProfile100 profile = new TrapezoidProfile100(8, 8, 0.001);
        double period = 0.02;
        double[] softLimits = new double[] { 0, 45 };
        // motor speed is rad/s
        Logger logger = Telemetry.get().rootLogger("foo");
        SimulatedMotor<Distance100> simMotor = new SimulatedMotor<>("test", logger, 600);
        SimulatedEncoder<Distance100> simEncoder = new SimulatedEncoder<>(
                "test", logger,
                simMotor,
                165, // see above
                -Double.MAX_VALUE,
                Double.MAX_VALUE);

        GravityServo g = new GravityServo(
                simMotor,
                "test", logger,
                pivotParams,
                pivotController,
                profile,
                period,
                simEncoder,
                softLimits);
        // start at zero
        assertEquals(0, g.getPosition().getAsDouble(), kDelta);
        // one second
        for (int i = 0; i < 50; ++i) {
            g.setPosition(1);
            stepTime(0.02);
        }
        // this overshoots a little, i think maybe because of the slight lag in
        // measurement.
        assertEquals(1, g.getPosition().getAsDouble(), kDelta);
    }

}

package org.team100.lib.motion.simple;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

/**
 * Produce a real or simulated subsystem depending on identity.
 */
public class SimpleSubsystemFactory {
    private final Profile100 profile;

    public SimpleSubsystemFactory() {
        profile = new TrapezoidProfile100(2, 2, 0.05);
    }

    public SimpleSubsystem get() {
        switch (Identity.instance) {
            case COMP_BOT:
                return getDefault();
            case BLANK:
                return simulated();
            default:
                return getDefault();
        }
    }

    private SimpleSubsystem getDefault() {
        FeedforwardConstants FeedforwardConstants = new FeedforwardConstants();
        PIDConstants pidConstants = new PIDConstants(1);
        MotorWithEncoder100<Distance100> motor = new Falcon6DriveMotor(
                "simple/drive",
                1,
                true,
                10,
                1,
                1,
                pidConstants,
                FeedforwardConstants);

        PIDController positionController = new PIDController(1, 0, 0);

        PositionServoInterface<Distance100> actuator = new PositionServo<>(
                "simple/position",
                motor,
                motor,
                10,
                positionController,
                profile,
                Distance100.instance);
        actuator.reset();
        return new SimpleSubsystem(actuator);
    }

    private SimpleSubsystem simulated() {
        // example with a lot of reduction, 1 m/s free speed
        SimulatedMotor<Distance100> motor = new SimulatedMotor<>("simple/drive", 1);
        Encoder100<Distance100> encoder = new SimulatedEncoder<>(
                "simple/encoder",
                motor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);

        PIDController positionController = new PIDController(1, 0, 0);
        PositionServoInterface<Distance100> actuator = new PositionServo<>(
                "simple/position",
                motor,
                encoder,
                10,
                positionController,
                profile,
                Distance100.instance);
        actuator.reset();
        return new SimpleSubsystem(actuator);
    }
}

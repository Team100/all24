package org.team100.lib.motion.simple;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.drive.FalconDriveEncoder;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleSubsystemFactory {
    private final Identity m_identity;
    private final Experiments m_experiments;

    public SimpleSubsystemFactory(Identity identity, Experiments experiments) {
        m_identity = identity;
        m_experiments = experiments;
    }

    public SimpleSubsystem get() {
        switch (m_identity) {
            default:
                return getDefault();
        }
    }

    private SimpleSubsystem getDefault() {
        FalconDriveMotor motor = new FalconDriveMotor("blarg", 1, 10, 1, 1);
        Encoder100<Distance> encoder = new FalconDriveEncoder("blarg", motor, 1);

        PIDController velocityController = new PIDController(1, 0, 0);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.06, 0.3, 0.025);

        VelocityServo<Distance> velocityServo = new VelocityServo<>(
                m_experiments,
                "blarg",
                motor,
                encoder,
                velocityController,
                feedforward);
        PIDController positionController = new PIDController(1, 0, 0);
        ChoosableProfile profile = new ChoosableProfile(
                2, 2, 0, ChoosableProfile.Mode.TRAPEZOID);
        PositionServo<Distance> actuator = new PositionServo<>(
                "example",
                velocityServo,
                encoder,
                10,
                positionController,
                profile,
                x -> x);
        return new SimpleSubsystem(actuator);
    }

}

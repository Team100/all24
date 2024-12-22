package org.team100.frc2024.motion.amp;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.NullProfile;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The pivot is independent from the feeder, so it's a separate subsystem.
 */
public class AmpPivot extends SubsystemBase implements Glassy {

    /**
     * The outboard velocity PID units are duty cycle per RPM, so tiny values are
     * normal
     */
    private static final double kOutboardP = 0.0002;
    private static final double kOutboardI = 0.0;
    private static final double kOutboardD = 0.0;
    private static final int kCurrentLimit = 30;
    private static final int kCanId = 2;
    /**
     * Jul 19 2024 found this to be wrong, it was 55 but looks like 70.
     * Final is 3.75 inch sprocket 48t driven by 16t, so 3:1.
     * Intermediate is mystery gears, close to 1:1.
     * Primary is 4:1 * 5:1 planetary
     * so 4*5*3=60, but it measures as 70, so the mystery gears must be something
     * like 18:22.
     */
    private static final double kGearRatio = 70;

    private final GravityServoInterface m_ampAngleServo;

    private Profile100 m_activeProfile = new NullProfile();

    public AmpPivot(LoggerFactory parent) {
        LoggerFactory child = parent.child(this);

        PIDController controller = new PIDController(2.0, 0, 0, TimedRobot100.LOOP_PERIOD_S);
        controller.setTolerance(0.02);
        controller.setIntegratorRange(0, 0.1);

        switch (Identity.instance) {
            case DISABLED:
                Feedforward100 ff = Feedforward100.makeArmPivot();
                PIDConstants pid = new PIDConstants(kOutboardP, kOutboardI, kOutboardD);
                CANSparkMotor motor = new NeoCANSparkMotor(
                        child,
                        kCanId,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        ff,
                        pid);
                RotaryMechanism mech = new SimpleRotaryMechanism(
                        child,
                        motor,
                        new CANSparkEncoder(child, motor),
                        kGearRatio);
                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        child,
                        3,
                        0.645439,
                        EncoderDrive.INVERSE);
                AngularPositionServo servo = new OnboardAngularPositionServo(
                        child,
                        mech,
                        encoder,
                        () -> m_activeProfile,
                        controller);
                servo.reset();
                m_ampAngleServo = new OutboardGravityServo(servo, 5.0, 0.0);
                break;
            default:
                // For testing and simulation
                double freeSpeedRad_S = 600;
                SimulatedBareMotor simMotor = new SimulatedBareMotor(
                        child, freeSpeedRad_S);
                RotaryMechanism simMech = new SimpleRotaryMechanism(
                        child,
                        simMotor,
                        new SimulatedBareEncoder(child, simMotor),
                        kGearRatio);
                RotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                        child, simMech);
                AngularPositionServo simServo = new OnboardAngularPositionServo(
                        child,
                        simMech,
                        simEncoder,
                        () -> m_activeProfile,
                        controller);
                simServo.reset();
                m_ampAngleServo = new OutboardGravityServo(simServo, 5.0, 0.0);
        }
    }

    public void setAmpState(Control100 state) {
        m_ampAngleServo.setState(state);
    }

    /** Zeros controller errors, sets setpoint to current position. */
    public void reset() {
        m_ampAngleServo.reset();
    }

    public void stop() {
        m_ampAngleServo.stop();
    }

    public OptionalDouble getPositionRad() {
        return m_ampAngleServo.getPositionRad();
    }

    public void setTorqueLimit(double torqueNm) {
        m_ampAngleServo.setTorqueLimit(torqueNm);
    }

    public void setProfile(Profile100 profile) {
        m_activeProfile = profile;
    }

    @Override
    public void periodic() {
        m_ampAngleServo.periodic();
    }

}

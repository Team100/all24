package org.team100.frc2024.motion.amp;

import java.util.OptionalDouble;

import org.team100.frc2024.motion.GravityServo2;
import org.team100.frc2024.motion.GravityServoInterface;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;

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

    private final SupplierLogger m_logger;
    private final GravityServoInterface m_ampAngleServo;

    public AmpPivot(SupplierLogger parent) {
        m_logger = parent.child(this);

        double period = 0.02;
        PIDController controller = new PIDController(2.0, 0, 0, period);
        controller.setTolerance(0.02);
        controller.setIntegratorRange(0, 0.1);

        switch (Identity.instance) {
            case COMP_BOT:
                Feedforward100 ff = Feedforward100.makeArmPivot();
                PIDConstants pid = new PIDConstants(kOutboardP, kOutboardI, kOutboardD);
                CANSparkMotor motor = new NeoCANSparkMotor(
                        m_logger,
                        kCanId,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        ff,
                        pid);
                RotaryMechanism mech = new RotaryMechanism(
                        m_logger,
                        motor,
                        new CANSparkEncoder(m_logger, motor),
                        kGearRatio);
                RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        m_logger,
                        3,
                        0.645439,
                        EncoderDrive.INVERSE);

                // m_ampAngleServo = new GravityServo(
                // mech,
                // m_logger,
                // controller,
                // period,
                // encoder);

                AngularPositionServo servo = new OnboardAngularPositionServo(
                        m_logger,
                        mech,
                        encoder,
                        10, // TODO: remove this
                        controller);
                // servo.setProfile(profile);
                servo.reset();

                m_ampAngleServo = new GravityServo2(servo, 5.0, 0.0);
                break;
            default:
                // For testing and simulation
                double freeSpeedRad_S = 600;
                SimulatedBareMotor simMotor = new SimulatedBareMotor(
                        m_logger, freeSpeedRad_S);
                RotaryMechanism simMech = new RotaryMechanism(
                        m_logger,
                        simMotor,
                        new SimulatedBareEncoder(m_logger, simMotor),
                        kGearRatio);
                RotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                        m_logger, simMech);

                // m_ampAngleServo = new GravityServo(
                //         simMech,
                //         m_logger,
                //         controller,
                //         period,
                //         simEncoder);

                AngularPositionServo simServo = new OnboardAngularPositionServo(
                        m_logger,
                        simMech,
                        simEncoder,
                        10, // TODO: remove this
                        controller);
                // servo.setProfile(profile);
                simServo.reset();

                m_ampAngleServo = new GravityServo2(simServo, 5.0, 0.0);
        }
    }

    public void setAmpPosition(double value) {
        m_ampAngleServo.setPosition(value);
    }

    public void setAmpState(State100 state) {
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
        m_ampAngleServo.setProfile(profile);
    }

    @Override
    public void periodic() {
        m_ampAngleServo.periodic();
    }

    @Override
    public String getGlassName() {
        return "Amp Pivot";
    }
}

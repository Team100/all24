package org.team100.frc2024.motion.climber;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoVortexCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements Glassy {
    /*******************************************
     * ALERT ALERT ALERT this current limit is uncalibrated.
     * 
     * The climber is rigged with ANSI #35 chain
     * 
     * https://www.peerchain.com/product/35r-roller-chain/
     * 
     * breaking strength is ~2500 lbs == ~11 kN.
     * 
     * Sprocket radius (below) is 0.022m, reduction is 45,
     * so breaking torque at the motor is 5.5 Nm, which is beyond the stall torque,
     * so it's not possible to break the chain with the motor.
     * 
     * In the past I think we broke the *hooks* which replaced the steel chain
     * plates with aluminum. The renewed design doubles the hook plates.
     * 
     * https://docs.revrobotics.com/brushless/neo/vortex
     * 
     * Lifting the 152-lb (max) robot with two climbers would take 676/2=338N
     * per side, which is only 0.165Nm, which is about 5% of the stall torque. The
     * stall current is 211 A, and 5% of that is about 10 A, so that's the "holding"
     * current.
     * 
     * To actually move the climber takes more than the holding current. Say we want
     * to move 0.2m with the same 338N gravity force, that would require about 68 J.
     * Say we wanted to do that in 1s, that would require 68 W. Assuming a bit of
     * sag, 68 W would require something like 7 amps.
     * 
     * There's also some friction.
     * 
     * A reasonable starting point would be 20: it shouldn't self destruct but it
     * should lift the robot.
     * 
     * TODO: hard stops, top and bottom.
     */
    private static final int kCurrentLimit = 20;

    /**
     * 15 tooth 35-series sprocket
     * https://wcproducts.info/files/frc/drawings/Web-%2335%20Double%20Hub%20Sprockets.pdf
     */
    private static final double kSprocketDiameterM = 0.045;
    /**
     * one 5:1 and one 9:1 stage
     * https://www.revrobotics.com/rev-21-2103/
     * https://www.revrobotics.com/rev-21-2129/
     */
    private static final double kReduction = 45;

    private final SupplierLogger m_logger;
    private final LinearMechanism m1;
    private final LinearMechanism m2;

    public ClimberSubsystem(SupplierLogger parent, int leftClimberID, int rightClimberID) {
        m_logger = parent.child(this);
        Util.warn("\n**** Uncalibrated climber current limit!!!  FIX THIS FOR COMP! ****\n");
        Util.warn("\n**** Uncalibrated climber polarity!!!  FIX THIS FOR COMP! ****\n");
        SupplierLogger leftLogger = m_logger.child("left");
        SupplierLogger rightLogger = m_logger.child("right");
        switch (Identity.instance) {
            case COMP_BOT:
                NeoVortexCANSparkMotor vp1 = new NeoVortexCANSparkMotor(
                        leftLogger,
                        leftClimberID,
                        MotorPhase.REVERSE,
                        kCurrentLimit,
                        Feedforward100.makeNeoVortex(),
                        new PIDConstants(0, 0, 0));
                m1 = new LinearMechanism(vp1,
                        new CANSparkEncoder(leftLogger, vp1),
                        kReduction,
                        kSprocketDiameterM);

                NeoVortexCANSparkMotor vp2 = new NeoVortexCANSparkMotor(
                        rightLogger,
                        rightClimberID,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        Feedforward100.makeNeoVortex(),
                        new PIDConstants(0, 0, 0));
                m2 = new LinearMechanism(vp2,
                        new CANSparkEncoder(rightLogger, vp2),
                        kReduction,
                        kSprocketDiameterM);
                break;
            default:
                // for testing and simulation
                // neo vortex free speed is 6784 rpm, 710 rad/s
                SimulatedBareMotor vs1 = new SimulatedBareMotor(leftLogger, 710);
                m1 = new LinearMechanism(vs1,
                        new SimulatedBareEncoder(leftLogger, vs1),
                        kReduction,
                        kSprocketDiameterM);

                SimulatedBareMotor vs2 = new SimulatedBareMotor(rightLogger, 710);
                m2 = new LinearMechanism(vs2,
                        new SimulatedBareEncoder(rightLogger, vs2),
                        kReduction,
                        kSprocketDiameterM);
        }
    }

    public void setLeftWithSoftLimits(double value) {
        OptionalDouble e1Position = m1.getPositionM();
        if (e1Position.isEmpty()) {
            m1.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() > 300 && value >= 0) {
            m1.setDutyCycle(0);
            return;
        }
        if (e1Position.getAsDouble() < 5 && value <= 0) {
            m1.setDutyCycle(0);
            return;
        }
        m_logger.logDouble(Level.TRACE, "LEFT VALUE", () -> value);
    }

    public void setRightWithSoftLimits(double value) {
        OptionalDouble e2Position = m2.getPositionM();
        if (e2Position.isEmpty()) {
            m2.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() > 300 && value >= 0) {
            m2.setDutyCycle(0);
            return;
        }
        if (e2Position.getAsDouble() < 5 && value <= 0) {
            m2.setDutyCycle(0);
            return;
        }
        m_logger.logDouble(Level.TRACE, "RIGHT VALUE", () -> value);
    }

    public void zeroClimbers() {
        m1.resetEncoderPosition();
        m2.resetEncoderPosition();
    }

    /** set left climb duty cycle */
    public void setLeft(double value) {
        m1.setDutyCycle(value);
    }

    /** set right climb duty cycle */
    public void setRight(double value) {
        m2.setDutyCycle(value);
    }

    public OptionalDouble getRightPosition() {
        return m2.getPositionM();
    }

    public OptionalDouble getLeftPosition() {
        return m1.getPositionM();
    }

    @Override
    public void periodic() {
        m_logger.logOptionalDouble(Level.TRACE, "climber 1 position (m)", m1::getPositionM);
        m_logger.logOptionalDouble(Level.TRACE, "climber 2 position (m)", m2::getPositionM);
        m_logger.logOptionalDouble(Level.TRACE, "climber 1 velocity (m_s)", m1::getVelocityM_S);
        m_logger.logOptionalDouble(Level.TRACE, "climber 2 velocity (m_s)", m2::getVelocityM_S);
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}

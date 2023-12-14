package org.team100.lib.motion.arm;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Arm mechanism with two joints.
 * 
 * 
 */
public class ArmSubsystem extends SubsystemBase {
    private static final double kFilterTimeConstantS = 0.06;
    private static final double kFilterPeriodS = 0.02;

    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final Motor100<Angle> m_lowerArmMotor;
    private final Motor100<Angle> m_upperArmMotor;
    private final Encoder100<Angle> m_lowerArmEncoder;
    private final Encoder100<Angle> m_upperArmEncoder;
    // zeros are measured in TURNS NOT RADIANS
    private final double m_lowerEncoderZero;
    private final double m_upperEncoderZero;
    private final ArmVisualization m_viz;

    private ArmAngles m_previousPosition;

    // use the factory to instantiate
    ArmSubsystem(
            String name,
            Motor100<Angle> lowerMotor,
            Encoder100<Angle> lowerEncoder,
            Motor100<Angle> upperMotor,
            Encoder100<Angle> upperEncoder,
            double lowerEncoderZero,
            double upperEncoderZero) {
        m_name = name;
        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(kFilterTimeConstantS, kFilterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(kFilterTimeConstantS, kFilterPeriodS);

        m_lowerArmMotor = lowerMotor;
        m_lowerArmEncoder = lowerEncoder;
        m_upperArmMotor = upperMotor;
        m_upperArmEncoder = upperEncoder;
        m_lowerEncoderZero = lowerEncoderZero;
        m_upperEncoderZero = upperEncoderZero;

        m_viz = new ArmVisualization(this);

        m_previousPosition = getPosition();
    }

    @Override
    public void periodic() {
        m_viz.periodic();
    }

    /** Arm angles (radians), 0 up, positive forward. */
    public ArmAngles getPosition() {
        ArmAngles result = new ArmAngles(
                m_lowerMeasurementFilter.calculate(getLowerArmAngleRadians()),
                m_upperMeasurementFilter.calculate(getUpperArmAngleRadians()));
        t.log(Level.DEBUG, m_name + "/Lower Encoder Pos: ", result.th1);
        t.log(Level.DEBUG, m_name + "/Upper Encoder Pos: ", result.th2);
        return result;
    }

    /** Joint velocities in radians per second. */
    public ArmAngles getVelocity() {
        ArmAngles position = getPosition();
        double th1 = position.th1 - m_previousPosition.th1;
        double th2 = position.th2 - m_previousPosition.th2;
        m_previousPosition = position;
        ArmAngles result = new ArmAngles(th1 * 50, th2 * 50);
        t.log(Level.DEBUG, m_name + "/Lower Encoder Vel: ", result.th1);
        t.log(Level.DEBUG, m_name + "/Upper Encoder Vel: ", result.th2);
        return result;
    }

    /**
     * Set motor controller duty cycle in range [-1, 1]
     * 
     * @param u1 lower, proximal
     * @param u2 upper, distal
     */
    public void set(double u1, double u2) {
        m_lowerArmMotor.setDutyCycle(u1);
        m_upperArmMotor.setDutyCycle(u2);
    }

    public void close() {
        m_lowerArmMotor.close();
        m_upperArmMotor.close();
        m_lowerArmEncoder.close();
        m_upperArmEncoder.close();
    }

    /** Lower arm angle (radians), 0 up, positive forward. */
    private double getLowerArmAngleRadians() {
        double posRad = m_lowerArmEncoder.getAbsolutePosition();
        double posTurn = posRad / (2 * Math.PI);
        double correctedPosTurn = (posTurn - m_lowerEncoderZero);
        return correctedPosTurn * 2 * Math.PI;
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArmAngleRadians() {
        double posRad = m_upperArmEncoder.getAbsolutePosition();
        double posTurn = posRad / (2 * Math.PI);
        double correctedPosTurn = (posTurn - m_upperEncoderZero);
        return correctedPosTurn * 2 * Math.PI;
    }

}

package org.team100.lib.motion.arm;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Arm mechanism with two joints.
 */
public class ArmSubsystem extends SubsystemBase implements Glassy  {
    private static final double kFilterTimeConstantS = 0.06;
    private static final double kFilterPeriodS = 0.02;

    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final Motor100<Angle100> m_lowerArmMotor;
    private final Motor100<Angle100> m_upperArmMotor;
    private final Encoder100<Angle100> m_lowerArmEncoder;
    private final Encoder100<Angle100> m_upperArmEncoder;

    private final ArmVisualization m_viz;

    private ArmAngles m_previousPosition;

    // use the factory to instantiate
    /**
     * 
     * @param name
     * @param lowerMotor
     * @param lowerEncoder  Lower arm angle (radians), 0 up, positive forward.
     * @param upperMotor
     * @param upperEncoder Upper arm angle (radians), 0 up, positive forward.
     */
    ArmSubsystem(
            String name,
            Motor100<Angle100> lowerMotor,
            Encoder100<Angle100> lowerEncoder,
            Motor100<Angle100> upperMotor,
            Encoder100<Angle100> upperEncoder) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);

        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(kFilterTimeConstantS, kFilterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(kFilterTimeConstantS, kFilterPeriodS);

        m_lowerArmMotor = lowerMotor;
        m_lowerArmEncoder = lowerEncoder;
        m_upperArmMotor = upperMotor;
        m_upperArmEncoder = upperEncoder;


        m_viz = new ArmVisualization(this);

        m_previousPosition = getPosition();
    }

    @Override
    public void periodic() {
        m_lowerArmMotor.periodic();
        m_upperArmMotor.periodic();
        m_viz.periodic();
    }

    /** Arm angles (radians), 0 up, positive forward. */
    public ArmAngles getPosition() {
        ArmAngles position = new ArmAngles(
                MathUtil.angleModulus(m_lowerMeasurementFilter.calculate(m_lowerArmEncoder.getPosition())),
                MathUtil.angleModulus(m_upperMeasurementFilter.calculate(m_upperArmEncoder.getPosition())));
        t.log(Level.TRACE, m_name, "position", position);
        return position;
    }

    /** Joint velocities in radians per second. */
    public ArmAngles getVelocity() {
        ArmAngles position = getPosition();
        double th1 = position.th1 - m_previousPosition.th1;
        double th2 = position.th2 - m_previousPosition.th2;
        m_previousPosition = position;
        ArmAngles velocity = new ArmAngles(th1 * 50, th2 * 50);
        t.log(Level.TRACE, m_name, "velocity", velocity);
        return velocity;
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

    @Override
    public String getGlassName() {
        return "Arm";
    }

    
}

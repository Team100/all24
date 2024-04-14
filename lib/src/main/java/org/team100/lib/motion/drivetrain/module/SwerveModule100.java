package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Feedforward and feedback control of a single module.
 */
public class SwerveModule100 implements Glassy {

    /**
     * To counteract drive torque coupling, the steering motor applies a negative
     * torque.
     */
    private static final double kDriveSteerCoupling = -0.1;

    protected static final double dt = 0.02;

    private final String m_name;
    private final VelocityServo<Distance100> m_driveServo;
    private final PositionServoInterface<Angle100> m_turningServo;
    private final SwerveModuleVisualization m_viz;

    public SwerveModule100(
            String name,
            VelocityServo<Distance100> driveServo,
            PositionServoInterface<Angle100> turningServo) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = Names.append(name, this);
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        m_viz = new SwerveModuleVisualization(this);
    }

    /**
     * Only SwerveModuleCollection calls this.
     */
    void setDesiredState(SwerveModuleState desiredState) {
        setRawDesiredState(SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningServo.getPosition())));
    }

    /**
     * Only SwerveModuleCollection calls this.
     * 
     * This is for testing only, it does not optimize.
     */
    void setRawDesiredState(SwerveModuleState state) {
        if (Double.isNaN(state.speedMetersPerSecond))
            throw new IllegalArgumentException("speed is NaN");
        if (Experiments.instance.enabled(Experiment.DriveSteerCouplingCompensation)) {
            m_driveServo.setVelocity(state.speedMetersPerSecond);
            m_turningServo.setPosition(state.angle.getRadians(), kDriveSteerCoupling * m_driveServo.getTorque());
        } else {
            m_driveServo.setVelocity(state.speedMetersPerSecond);
            m_turningServo.setPosition(state.angle.getRadians());
        }

    }

    /** For testing */
    SwerveModuleState getDesiredState() {
        return new SwerveModuleState(
                m_driveServo.getSetpoint(),
                new Rotation2d(m_turningServo.getGoal()));
    }

    /** Make sure the setpoint and measurement are the same. */
    public void reset() {
        m_turningServo.reset();
        m_driveServo.reset();
    }

    /** for testing only */
    State100 getSetpoint() {
        return m_turningServo.getSetpoint();
    }

    /**
     * Only SwerveModuleCollection calls this.
     */
    void periodic() {
        m_viz.periodic();
    }

    public void close() {
        m_turningServo.close();
    }

    public String getName() {
        return m_name;
    }

    @Override
    public String getGlassName() {
        return "SwerveModule100";
    }

    /////////////////////////////////////////////////////////////
    //
    // Package private for SwerveModuleCollection
    //

    /** @return current measurements */
    SwerveModuleState getState() {
        return new SwerveModuleState(m_driveServo.getVelocity(), new Rotation2d(m_turningServo.getPosition()));
    }

    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveServo.getDistance(), new Rotation2d(m_turningServo.getPosition()));
    }

    boolean atSetpoint() {
        return m_turningServo.atSetpoint();
    }

    boolean atGoal() {
        return m_turningServo.atGoal();
    }

    void stop() {
        m_driveServo.stop();
        m_turningServo.stop();
    }
}

package org.team100.frc2023.commands.arm;

import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.frc2023.subsystems.arm.ArmInterface;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmTrajectory extends Command {
    public static class Config {
        public double oscillatorFrequencyHz = 2;
        /** amplitude (each way) of oscillation in encoder units */
        public double oscillatorScale = 0.025;
        /** start oscillating when this close to the target. */
        public double oscillatorZone = 0.1;
        public TrajectoryConfig safeTrajectory = new TrajectoryConfig(9, 1.5);
        public TrajectoryConfig normalTrajectory = new TrajectoryConfig(12, 2);
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final ArmInterface m_arm;
    private final ArmPosition m_position;
    private final boolean m_oscillate;

    private final Timer m_timer;

    private Trajectory m_trajectory;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     */
    public ArmTrajectory(ArmPosition position, ArmInterface arm, boolean oscillate) {
        m_arm = arm;
        m_position = position;
        m_oscillate = oscillate;
        m_timer = new Timer();

        addRequirements(m_arm.subsystem());
    }

    @Override
    public void initialize() {
        m_timer.restart();
        final TrajectoryConfig trajectoryConfig;
        if (m_position == ArmPosition.SAFE) {
            trajectoryConfig = m_config.safeTrajectory;
            m_arm.setControlSafe();
        } else {
            trajectoryConfig = m_config.normalTrajectory;
            m_arm.setControlNormal();
        }
        m_trajectory = new ArmTrajectories(trajectoryConfig).makeTrajectory(
                m_arm.getMeasurement(),
                m_position,
                m_arm.getCubeMode());
    }

    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        ArmAngles measurement = m_arm.getMeasurement();
        double currentUpper = measurement.th2;
        double currentLower = measurement.th1;

        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);

        double desiredUpper = desiredState.poseMeters.getX();
        double desiredLower = desiredState.poseMeters.getY();

        double upperError = desiredUpper - currentUpper;

        if (m_oscillate && upperError < m_config.oscillatorZone) {
            desiredUpper += oscillator(curTime);
        }

        ArmAngles reference = new ArmAngles(desiredLower, desiredUpper);

        m_arm.setReference(reference);

        t.log("/Arm Trajec/measurmentX", currentUpper);
        t.log("/Arm Trajec/measurmentY", currentLower);
        t.log("/Arm Trajec/Setpoint Upper", desiredUpper);
        t.log("/Arm Trajec/Setpoint Lower", desiredLower);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setControlNormal();
        m_arm.setReference(m_arm.getMeasurement());
    }

    @Override
    public boolean isFinished() {
        if (m_position == ArmPosition.SAFEWAYPOINT) {
            return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
        }
        return false;
    }

    private double oscillator(double timeSec) {
        return m_config.oscillatorScale * Math.sin(2 * Math.PI * m_config.oscillatorFrequencyHz * timeSec);
    }

}

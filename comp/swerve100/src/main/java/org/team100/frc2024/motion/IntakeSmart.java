package org.team100.frc2024.motion;

import org.team100.frc2024.SensorInterface;
import org.team100.frc2024.motion.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSmart extends Command {
    private final SensorInterface m_sensors;
    private final Intake m_intake;
    
    private boolean finished = false;

    public IntakeSmart(SensorInterface sensors, Intake intake) {
        m_sensors = sensors;
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.intake();
    }

    @Override
    public void execute() {
        if (m_sensors.getFeederSensor()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

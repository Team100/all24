package org.team100.lib.commands.arm;


import org.team100.lib.motion.arm.ArmSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmTrajectoryCommand extends Command {

    private boolean definedAngle = true;
    private final ArmSubsystem m_armSubsystem;
    private final double m_startAngle;
    private final double m_endAngle;
    private final Timer m_timer;
    private final Translation2d m_set;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     * Units for angles are degrees
     */
    public ArmTrajectoryCommand(ArmSubsystem armSubSystem, Translation2d set, double startAngle, double endAngle) {
        m_set = set;
        m_armSubsystem = armSubSystem;
        m_endAngle = endAngle;
        m_startAngle = startAngle;
        m_timer = new Timer();
        addRequirements(m_armSubsystem);
    }

    public ArmTrajectoryCommand(ArmSubsystem armSubSystem, Translation2d set) {
        this(armSubSystem, set, -1, -1);
        definedAngle = false;
    }

    @Override
    public void initialize() {
        m_timer.restart();
        if (definedAngle) {
            m_armSubsystem.setTrajectory(m_timer,m_set,m_startAngle,m_endAngle);
        } else {
            m_armSubsystem.setTrajectory(m_timer,m_set);
        }

    }

    @Override
    public boolean isFinished() {
        double toleranceRadians = 0.02;
        if (Math.abs(m_armSubsystem.getTrajectoryError().th1) < toleranceRadians && Math.abs(m_armSubsystem.getTrajectoryError().th2) < toleranceRadians) {
            return true;
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.endTrajectory();
    }
}
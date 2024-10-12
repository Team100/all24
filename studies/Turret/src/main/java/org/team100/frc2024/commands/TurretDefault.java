package org.team100.frc2024.commands;

import java.util.function.Supplier;

import org.team100.frc2024.turret.Turret;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControl.Velocity;

import edu.wpi.first.wpilibj2.command.Command;

public class TurretDefault extends Command {
    private final Turret m_turret;
    private final Supplier<DriverControl.Velocity> m_supplier;

    public TurretDefault(
            Supplier<DriverControl.Velocity> twistSupplier,
            Turret turret) {
        m_turret = turret;
        m_supplier = twistSupplier;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Velocity velocity = m_supplier.get();
        if (Math.abs(velocity.y()) > 0.25 || Math.abs(velocity.x()) > 0.25) {
         m_turret.setAngle(Math.atan2(velocity.y(), -1.0 * velocity.x()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }
}

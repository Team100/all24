package org.team100.frc2024;

import org.team100.lib.telemetry.Logger;

public class TestLogger24 implements Logger {
    @Override
    public boolean enabled() {
        return false;
    }

	@Override
	public Logger child(String stem) {
		return this;
	}
}

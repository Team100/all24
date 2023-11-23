package org.team100.lib.experiments;

import org.team100.lib.config.Identity;

public class MockExperiments extends Experiments {
    public boolean enablement = true;

    public MockExperiments() {
        super(Identity.BLANK);
    }

    @Override
    public boolean enabled(Experiment experiment) {
        return enablement;
    }

}

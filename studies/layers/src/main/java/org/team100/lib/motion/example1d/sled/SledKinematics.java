package org.team100.lib.motion.example1d.sled;

import org.team100.lib.motion.example1d.framework.Kinematics;

public class SledKinematics implements Kinematics<SledWorkstate, SledConfiguration> {

    @Override
    public SledWorkstate forward(SledConfiguration x) {
        return new SledWorkstate(x.getPositionM());
    }

    @Override
    public SledConfiguration inverse(SledWorkstate x) {
        return new SledConfiguration(x.getState());
    }
    
}

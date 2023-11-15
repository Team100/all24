package org.team100.lib.motion.example1d.sled;

public class SledKinematics {

    public SledWorkstate forward(SledConfiguration x) {
        return new SledWorkstate(x.getPositionM());
    }

    public SledConfiguration inverse(SledWorkstate x) {
        return new SledConfiguration(x.getState());
    }

}

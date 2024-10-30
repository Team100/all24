package org.team100.frc2024.shooter.indexer;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;

public class IndexerCollection {

    public static Indexer get(
            LoggerFactory parent
            ) {
        switch (Identity.instance) {
            case DEMO_BOT:
                return new IndexerServo(parent, MotorPhase.REVERSE, 0);
            case BLANK:
            default:
                return new Indexer() {

                    @Override
                    public void set(double value) {
                        // TODO Auto-generated method stub
                        throw new UnsupportedOperationException("Unimplemented method 'set'");
                    }

                    @Override
                    public void stop() {
                        // TODO Auto-generated method stub
                        throw new UnsupportedOperationException("Unimplemented method 'stop'");
                    }
                    
                };
        }
    }
}

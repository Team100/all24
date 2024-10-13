package org.team100.frc2024.shooter.indexer;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;

public class IndexerCollection {

    public static Indexer get(
            LoggerFactory parent
            ) {
        switch (Identity.instance) {
            case BETA_BOT:
                return new IndexerServo(parent, 0);
            case BLANK:
            default:
                return new Indexer() {

                    @Override
                    public void set(double value) {
                        // TODO Auto-generated method stub
                        throw new UnsupportedOperationException("Unimplemented method 'set'");
                    }

                    @Override
                    public void setAngle(double value) {
                        // TODO Auto-generated method stub
                        throw new UnsupportedOperationException("Unimplemented method 'setAngle'");
                    }

                    @Override
                    public double getAngle() {
                        // TODO Auto-generated method stub
                        throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
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

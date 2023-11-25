package org.team100.lib.motion.crank;

public class ConfigurationConstant implements Configurations {
    private final Configuration m_conf;

    public ConfigurationConstant(Configuration conf) {
        m_conf = conf;
    }

    @Override
    public Configuration get() {
        return m_conf;
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}

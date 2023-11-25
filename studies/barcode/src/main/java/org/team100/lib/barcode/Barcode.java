package org.team100.lib.barcode;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Encapsulates the code reader. */
public class Barcode implements IntSupplier {
    private final DigitalOutput[] m_outputs;
    private final Mux mux;
    private final AnalogInput m_input;
    private final DigitalOutput m_odd;
    private final DigitalOutput m_even;
    private final Sensor m_array;

    private int id;

    public Barcode() {
        m_outputs = new DigitalOutput[] {
                new DigitalOutput(0),
                new DigitalOutput(1),
                new DigitalOutput(2),
                new DigitalOutput(3)
        };
        mux = new Mux(m_outputs);
        m_input = new AnalogInput(0);
        m_odd = new DigitalOutput(5);
        m_even = new DigitalOutput(4);

        // TODO: tune the thresholds
        double[] thresholds = new double[] { 2.5, 3, 3, 2.4, 2.5 };
        m_array = new Sensor(
                m_input,
                thresholds,
                mux,
                m_odd,
                m_even);
    }

    public void robotInit() {
        // TODO: represent failure in a better way than -1.
        id = m_array.readValue().orElse(-1);
    }

    public void teleopPeriodic() {
        System.out.printf("%d   %5.3f\n",
                m_array.readValue().getAsInt(),
                m_input.getAverageVoltage());
    }

    public void testPeriodic() {
        mux.set(1);
        try {
            Thread.sleep(5, 0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } // led driver startup time
        SmartDashboard.putNumber("Real 1", m_input.getAverageVoltage());

        mux.set(0);
        try {
            Thread.sleep(5, 0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        SmartDashboard.putNumber("Real 0", m_input.getAverageVoltage());
    }

    public void close() {
        for (int i = 0; i < m_outputs.length; ++i) {
            m_outputs[i].close();
        }
        m_input.close();
        m_odd.close();
        m_even.close();
    }

    @Override
    public int getAsInt() {
        return id;
    }

}

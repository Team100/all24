package org.team100.app;

import org.team100.log_annotation.Log;
import org.team100.log_annotation.Registrar;
import org.team100.log_annotation.Level;

/** Example class for log annotations. */
public class SomeClass {
    @Log(level = Level.INFO)
    private double someField;

    public SomeClass(Registrar registrar) {
        someField = 1;
        registrar.register(this);
    }

    @Log()
    private double someFunction() {
        return 2;
    }

}

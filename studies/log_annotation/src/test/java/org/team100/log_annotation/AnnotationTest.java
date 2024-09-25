package org.team100.log_annotation;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.app.SomeClass;

class AnnotationTest {
    @Test
    void testSimple() {
        Registrar registrar = new Registrar();
        new SomeClass(registrar);
        assertEquals(2, registrar.suppliers.size());
        assertEquals(1, registrar.suppliers.get("someField").getAsDouble());
        assertEquals(2, registrar.suppliers.get("someFunction").getAsDouble());
    }
}

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AppTest {

    @Test
    void testApp() {
        App app = new App();
        assertEquals("asdf", app.foo());
    }
}

# Monitor

This contains monitoring and self-testing code that we have never used.

Maybe we can come back to it if a student takes interest.

To hook it up, wire it into the container:

```java
// these should be fields
final MorseCodeBeep m_beep;
final Monitor m_monitor;

// 20 words per minute is 60 ms.
m_beep = new MorseCodeBeep(0.06);
// m_beep = new Beep();
BooleanSupplier test = () -> driverControl.annunicatorTest() || m_beep.getOutput();
m_monitor = new Monitor(monitorLogger, new Annunciator(6), test);
// The monitor runs less frequently than the control loop.
robot.addPeriodic(m_monitor::periodic, 0.2, "monitor");
```

The self test harness is a command that should be scheduled in testInit().

To instantiate it, you need to give it an enable condition, e.g. 

```java
m_selfTest = new SelfTestRunner(this, operatorControl::selfTestEnable);
```

The morse code thing uses a "message."
```java
MorseCodeBeep beep = m_robotContainer.m_beep;
beep.setDuration(1);
beep.setMessage("K");
CommandScheduler.getInstance().schedule(beep);
        ```
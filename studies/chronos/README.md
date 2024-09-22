# Chronos

an idea for fine-grained selective profiling that we might want to get back to at some point

make it a member of TimedRobot100:

```java
private final Chronos chronos;
```

wire it up in TimedRobot100's constructor:

```java
chronos = new Chronos(m_logger);
addPeriodic(chronos::dump, period, "chronos output");
```

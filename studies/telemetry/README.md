# Telemetry

A simple wrapper for Network Tables logging.

The logged quantities will appear in both Glass and the USB log.

This is intended to be __very__ simple and limited, to reduce the verbosity
of the most-common case.

If you want to log something in a class, put this at the
top of the class:

```
    private final Telemetry t = Telemetry.get();
```

And at the logging site, imagine you have a local variable
called something like ```foo```:

```
    t.log("/key/with/slashes", foo);
```

You don't need to mess with Network Tables directly at all.

You don't need to implement Sendable or make unnecessary
member variables just to log them.

You __do__ need to be careful with the keys:
make them make sense.  You'll be searching for them
in vscode, so make them easy to find.

The slashes turn into nesting in Glass.

See Robot.java for an example.
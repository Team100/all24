# Primitive

Primitive loggers are the low-level transport for base types like double and String.

Nothing here should be used by client code.

There are two types of primitive loggers:

* Network Tables logging: similar to how we've always done it; limited in scale
* UDP logging with a custom protocol: much faster and not entirely reliable

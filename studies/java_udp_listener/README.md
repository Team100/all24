# Java UDP Listener

Our usual python coprocessor code is good at managing bulk operations, like processing a whole frame of camera data, but it's _really_ _really_ bad at fine-grained operations like decoding log messages, like 60X worse.

Java, on the other hand, is pretty good at those fine-grained operations, so for the log listener, the coprocessor will run a jar.

This is our first Raspberry Pi jar project. :-( :-)

# Building

To build this project, use the gradle extension and click "shadowJar".

Alternatively, in this directory, type

```
./gradlew shadowjar
```

The executable will appear in build/libs.

Note this appears to build an x64 version on my machine, is cross-compilation a thing?  Maybe build it on the pi?

This build process needs work.

At the moment this pulls in all the robot dependencies from lib, which is way more than we want.

To fix that, there needs to be a "logging lib" that is separate from lib.

# See also

WPILibPi/deps/examples/java-multiCameraServer

wpilibsuite/StandaloneAppSamples/Java/build.gradle

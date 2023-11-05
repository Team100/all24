# LQR with Angles

This has turned into a rewrite of controls.

Note the build.gradle changes to make roadrunner profiles work.

```
repositories {
    maven {
        url = 'https://maven.brott.dev/'
    }
}
dependencies {
    ...
    implementation 'com.acmerobotics.roadrunner:core:0.5.6'
}
```
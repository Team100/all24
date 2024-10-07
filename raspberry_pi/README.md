# Raspberry Pi

This is for the 2025 season, not ready yet.

Team 100 uses Raspberry Pis for coprocessing, with several different workloads:

* AprilTag vision
* Game piece vision
* Friend/foe vision
* Gyroscope
* Logging UDP listener
* Localization using GTSAM

All these different workloads use the same deployed artifact, built out of this directory, using the board identity to select behavior.

The key is that python can load modules out of zip files, so you zip up the source directory and upload it. There's also a very short python script, runapp.py, that runs the zip file -- you just upload this one time.

To best use this idea, install the vscode Gradle extension (from Microsoft). It will make available a Gradle "activity" which includes the task "distribution/distZip". Click the little play button, and it will produce a "zip" file containing the "app" code. To make it easier to find, you can "pin" the task.

Find the zip file in build/distributions/app.zip. Upload it to the RPi, next to runapp.py.

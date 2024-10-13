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

# How to run locally

From the command line in the raspberry_pi directory:

```
python3 runapp.py
```

Then look at http://localhost:1181/

And you should see the test image being processed over and over.


# How to deploy

Install the vscode Gradle extension (from Microsoft). It will make available a Gradle "activity" which includes the task "distribution/distZip". Click the little play button, and it will produce a "zip" file containing the "app" code. To make it easier to find, you can "pin" the task.

Find the zip file in build/distributions/app.zip. Upload it to the RPi, next to runapp.py.

# How to run the tests

There are two ways to run the tests:

Install the vscode python extension, and a little Erlenmeyer flask will appear in the toolbar on the left; this scans the source tree for test methods, and makes a little list of them.  You can run all of them, or individual ones.

The same extension also puts a little marker in the file editor, on the left side of each test method, and provides test output there too, in case of failures.

You can also run all the tests from the command line from the raspberry_pi directory:

```
python3 runtests.py
```

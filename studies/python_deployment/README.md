# Python Deployment

This is an experiment to make multi-file python work better with the Raspberry Pi's, without creating a bunch of complexity.

The key is that python can load modules out of zip files.

So you zip up the source directory and upload it.

You also have a very short python script that runs the zip file -- you just upload this one time.

To best use this idea, install the vscode Gradle extension (from Microsoft). It will make available a Gradle "activity" which includes the task "distribution/distZip". Click the little play button, and it will produce a "zip" file containing the "app" code. To make it easier to find, you can "pin" the task.

Find the zip file in build/distributions/app.zip. Upload it to the RPi, next to runapp.py.

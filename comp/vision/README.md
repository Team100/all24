# Vision

What's here:

* led-timer.py: exploration by @Vasili into image capture pipeline latency

TODO: clean up the stuff here.



OLDER BELOW

-------------------------------------

# AprilTags Example

What's here:


* app4.py: capture images, detect apriltags, publish on the new networktables four using msgpack.  no dependency on draw.py.  uses pyntcore instead of pynetworktables python library, since pynetworktables doesn't support nt4.

# More about the example

Sample AprilTags detector and pose estimator.
Works on unix-like platforms; the AprilTags developers don't use Windows
and so it's a pain to set up on Windows (I failed to do it).

The Raspberry Pi is what we'll actually be using on the real robot, so it's
the best choice.


# Install and Run on WPILibPi Vision Application

The WPILibPi framework includes a "Vision Application" that can be manipulated
using the web service.  To set it up requires a few steps:

First make a fork of github.com/Team100/main2023 into your own account, then
clone it onto your laptop, e.g. using git bash or vscode or however you prefer.

Attach the pi to the LAN, e.g. with an ethernet switch.

From your laptop browser, browse to http://wpilibpi.local/ and click the "writeable" button on the top.

Now install some software on the pi, by ssh'ing to it (e.g. using PuTTY or whatever
ssh client you want, using username 'pi' and password 'raspberry'), and execute these:

```
sudo date -s '15 Sep 2022 14:25' (use the actual date)
sudo apt update
sudp apt upgrade
python3 -m  pip install --upgrade pip setuptools wheel numpy
python3 -m pip install pupil-apriltags
sudo /sbin/reboot now
```

When the pi finishes restarting, go to http://wpilibpi.local/ again and click the "writeable" button on the top.

Now we're going to install the little library for AprilTag pose rendering.

Click "Application" on the left side, and then in the second box called "File Upload," click "Choose File"
and find main2023/apriltags_example/draw.py.  Then click "Upload."

Finally, install the app itself.

Still in the "Application" section, in the first box called "Vision Application Configuration," click "Choose File"
and find main2023/apriltags_example/app.py.  Then click "Upload."

Then navigate to "Vision Status" on the left and click "Down" and then "Up" to restart the service.

Then navigate to "Vision Settings" and click "Open Stream".  you should see wpilibpi.local:1181/, which
will render a bunch of stuff about the camera, and a little video stream showing what the camera sees.

Rewrite the url to the next port, wpilibpi.local:1182 (note the "2"), and you should see the *output*
of the app, which renders little boxes on the AprilTags it finds, as well as the frame rate.

The app also publishes the id and pose of any AprilTags it finds to NetworkTables.


# Install and Run on WPILibPi Raspberry Pi Command Line

To do AprilTag detection via the command line, there are several steps:

First make a fork of github.com/Team100/main2023 into your own account; you'll pull from it below.

Then attach the pi to a monitor, mouse, and keyboard.
Attach the pi to the LAN, e.g. with an ethernet switch.
Turn it on.

The pi boots up in "read only" mode, so we have to change that to allow writing.  From
your laptop browser, browse to http://wpilibpi.local/ and click the "writeable" button on the top.

The attached display should show text. Login using username 'pi' and password 'raspberry'.

First we have to fix the clock, or none of the steps below will work.  Use whatever the actual time is.

```
sudo date -s '15 Sep 2022 14:25'
```

Next we'll add some software.  This takes a long time, like 30 minutes, waiting for the pi to download everything.
From the pi command line, type these:


```
sudo apt update
sudp apt upgrade
sudo apt install lightdm
sudo apt install raspberrypi-ui-mods
sudo apt install git
sudo apt install chromium-browser  (not strictly required but handy!)
python3 -m pip install pupil-apriltags
git clone https://github.com/[your account]/main2023.git
sudo /sbin/reboot now
```

After rebooting, you have to make it "writeable" again, as above: use your laptop to browse to http://wpilibpi.local/ and click the "writeable" button on the top.
We also have to get the WPI stuff to leave the camera alone for now: click "Vision Status" on the left side and click "Down" to stop the vision service.
Click "Vision Settings" on the left and then "remove" if there's a camera listed there.

Now the monitor should show a graphical login screen.  Login with pi/raspberry as before.

Open a terminal window by clicking the icon in the menu bar on the upper left.

```
cd main2023/apriltags
python3 pic.py   (for the canned image)
python3 vid.py   (for the video camera)
```

Point the camera at some AprilTags and rejoice!

# Install and Run on Ubuntu

On my Ubuntu workstation, installation is super easy.  First install the pupil-apriltags python wrapper (which also installs apriltags itself):

```
python3 -m pip install pupil-apriltags
```

Then make a fork of github.com/Team100/main2023 into your own account, and then clone it somewhere on your workstation:

```
cd <somewhere handy>
git clone git@github.com:<your account>/main2023.git
cd main2023/apriltags_example
```

To see detections in a single image:

```
python3 pic.py
```

To see detections in your webcam's video stream:

```
python3 vid.py
```

# Details

The pose estimator and rendering steps make some assumptions about the camera focal length, which are
surely wrong.

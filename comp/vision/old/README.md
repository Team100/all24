# Vision

What's here:

* led_timer.py: exploration by @Vasili into image capture pipeline latency
* gamepiece_finder.py: color-based cube- and cone-finder using opencv
* blip_serializer.py: find retroreflective poles for 2023
* tag_finder.py: find apriltags as used in 2023


# How to set up the Raspberry Pi

* Make sure you're using a Raspberry Pi 4b or 5; the 3 won't work.
* Grab the image from the [WPILibPi Releases page](https://github.com/wpilibsuite/WPILibPi/releases)
* In ubuntu:
  * uncompress the zip file
  * find the img file in the file browser
  * right click and choose "open with disk image writer"
  * choose the destination sd card.  make sure you choose the right disk from the list.
  * click "start restoring" and confirm
* connect it to the internet and to your computer, e.g. through ethernet.
it's not sufficient to connect directly to your laptop at this stage, the pi needs access to the internet.
* open a browser to wpilibpi.local and click "writeable"
* ssh to it (pi@wpilibpi.local, password raspberry) and do:

```
sudo date -s '14 Nov 2023 08:31'
sudo apt update
sudo apt upgrade
python3 -m pip install --upgrade pip setuptools wheel numpy
python3 -m pip install pupil-apriltags msgpack
sudo /sbin/reboot now
```
* go to the browser at wpilibpi.local and click "writeable" again
* click "application" on the left side
* choose "uploaded python file" from the menu near the top
* choose all24/comp/vision/tag_finder.py
* click "upload and save"
* choose "vision settings" on the left side
* click "open stream", the green button on the right

you should see a small monochrome image with fps and timestamp, and if there are any apriltags in the image, they should be highlighted with id and coordinates.

# Details

The pose estimator and rendering steps make some assumptions about the camera focal length, which are
surely wrong.

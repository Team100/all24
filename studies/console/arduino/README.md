# Arduino

This supports a modular operator console, where each subconsole has its own arduino controller.

Each subconsole is its own Arduino "sketch," and there's a "library" for USB communication and
to manage names and ID's.

To use the Arduino IDE with this project you need to set the "sketchbook" to this directory.
You'll also need to edit the "library" code with a separate Arduino IDE from the "sketch" code,
because the Arduino folk don't want sketch editors unwittingly changing library code.

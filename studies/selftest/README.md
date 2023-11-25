# Selftest

The selftest system works by running two commands in parallel:
the command under test, and a command that makes observations
and executes assertions on those observations.  if any of the
assertions fail, the test is over.

Examples of tests we should have:

* run a trajectory and verify time and position along the way.
* provide simulated manual input and verify the response to it
* measure a few things, like battery voltage

Notes

to see how the colored text works, see
https://stackoverflow.com/questions/5762491/how-to-print-color-in-console-using-system-out-println

The ascii art is from here https://www.asciiart.eu/text-to-ascii-art
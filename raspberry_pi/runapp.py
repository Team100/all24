"""
Runs the code in the zip file (or the filesystem).

This should be uploaded to WPILibPi once, in the "Application
Configuration" tab, using the "Application" upload as an
"Uploaded Python file."

It works by adding the zip file to the import path.  Python can decode
the zip file on the fly, treating it just like a file system.

If it can't find the zip file, it looks in the local filesystem
(i.e. maybe you're running it from vscode), and if that doesn't work,
it exits with an error.

This file can also be run from the command line, or in vscode by clicking
the little triangle on the upper right (up there ^^^).
"""

import os
import sys
import traceback

if os.path.isfile("app.zip"):
    print("using the zip file")
    sys.path.insert(0, "app.zip")
else:
    print("using the local filesystem")

try:
    from app import main
except ImportError as e:
    print("".join(traceback.format_exception(e)))
    sys.exit("app import failed!")

main.main()

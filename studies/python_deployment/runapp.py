""" Runs the code in the zip file.

This should be uploaded to WPILibPi once, in the "Application
Configuration" tab, using the "Application" upload as an
"Uploaded Python file."

It works by adding the zip file to the import path.  Python can decode
the zip file on the fly, treating it just like a file system.

If it can't find the zip file, it looks in the local filesystem
(i.e. maybe you're running it from vscode), and if that doesn't work,
it exits with an error.
"""

import os
import sys

if os.path.isfile("app.zip"):
    print("using the zip file")
    sys.path.insert(0, "app.zip")
else:
    print("using the local filesystem")

try:
    from app import main
except ImportError:
    sys.exit("app import failed!")

main.main()

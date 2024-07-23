""" runs the code in the zip file """
import os
import sys
if os.path.isfile('app.zip'):
    # resolve the import using zip file
    sys.path.insert(0, "app.zip")
    print(sys.path[0])

from app import example1
from app import example2

example1.do_example1()
example2.do_example2()

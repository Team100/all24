import unittest

loader = unittest.TestLoader()
suite = loader.discover("tests", pattern="*test.py")

runner = unittest.TextTestRunner(verbosity=2)
runner.run(suite)
